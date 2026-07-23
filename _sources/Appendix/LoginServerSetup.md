# ECE 387 Login Server & Master Computer Setup

## Architecture Overview

```
                    ┌──────────────────────────────────┐
                    │   Login Server (NUC 9 or NUC 11) │
                    │   Ubuntu 24.04                   │
                    │   - OpenLDAP (user accounts)     │
                    │   - SSSD (authentication)        │
                    │   - NFS (shared home directories)│
                    └────────────┬─────────────────────┘
                                 │ Lab Ethernet
              ┌──────────────────┼──────────────────┐
              │                  │                  │
    ┌─────────┴──────┐  ┌────────┴───────┐  ┌──────┴─────────┐
    │  Master 01     │  │  Master 02     │  │  Master N...   │
    │  NUC 9         │  │  NUC 9         │  │  NUC 9         │
    │  Ubuntu 24.04  │  │  Ubuntu 24.04  │  │  Ubuntu 24.04  │
    │  ROS2 Jazzy    │  │  ROS2 Jazzy    │  │  ROS2 Jazzy    │
    │  SSSD client   │  │  SSSD client   │  │  SSSD client   │
    │  NFS client    │  │  NFS client    │  │  NFS client    │
    └────────┬───────┘  └────────┬───────┘  └────────┬───────┘
             │ Internal WiFi (AP)                      │
    ┌────────┴───────┐                       ┌────────┴───────┐
    │  Robot 01      │                       │  Robot N...    │
    │  RPi 4         │                       │  RPi 4         │
    │  Ubuntu 22.04  │                       │  Ubuntu 22.04  │
    │  ROS2 Humble   │                       │  ROS2 Humble   │
    └────────────────┘                       └────────────────┘
```

**Key concept:** Student home directories live on the login server and are NFS-mounted on every master. When a student logs in to any master, their home directory (`.bashrc`, SSH keys, ROS workspace) is automatically available — no per-machine setup needed.

> **Why not FreeIPA?** `freeipa-server` is not packaged for Ubuntu 24.04. OpenLDAP + SSSD is the standard replacement and provides the same centralized login and shared home directory functionality.

---

## Network Planning

Assign static IPs on your lab's Ethernet network before starting. All machines must be on the same subnet so they can reach each other.

| Host         | Hostname       | IP              |
|--------------|----------------|-----------------|
| Login Server | `ece387server` | `192.168.1.10`  |
| Master 01    | `master01`     | `192.168.1.101` |
| Master 02    | `master02`     | `192.168.1.102` |
| ...          | ...            | ...             |
| Master 14    | `master14`     | `192.168.1.114` |

> Adjust the subnet to match your lab's actual network. This guide uses the domain `ece387.local`.

---

## Part 1: Login Server Setup

### 1.0 Create a Working Directory

All scripts and configuration files for this setup are saved here so you can reference or reuse them later.

```bash
mkdir -p ~/Documents/ece387
```

### 1.1 Install Ubuntu 24.04 Server

1. Boot from Ubuntu 24.04 Server ISO.
2. During install, set:
   - Hostname: `ece387server`
   - Create a local admin account (e.g., `stanbaek`) — this is your personal admin account, separate from student accounts
   - Enable OpenSSH Server so you can manage the server remotely from your office
3. After install, set a static IP so the server is always reachable at the same address:

```bash
# Find your ethernet interface name (look for something like enp86s0 or eno1)
ip a

sudo nano /etc/netplan/00-installer-config.yaml
```

```yaml
network:
  version: 2
  ethernets:
    enp86s0:                      # replace with your actual interface name
      dhcp4: false
      addresses: [192.168.1.10/24]
      routes:
        - to: default
          via: 192.168.1.1        # your lab gateway/router IP
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

```bash
# Apply the network config — takes effect immediately without rebooting
sudo netplan apply
```

### 1.2 Set Hostname and /etc/hosts

The hostname tells the machine its own name. `/etc/hosts` maps that name to an IP address locally, which is required for LDAP to work correctly — LDAP uses the hostname to identify the server, and it must match the IP.

```bash
# Set the system hostname
sudo hostnamectl set-hostname ece387server
```

```bash
sudo nano /etc/hosts
```

The file should contain:

```
127.0.0.1   localhost
192.168.1.10  ece387server.ece387.local  ece387server
```

> `ece387server.ece387.local` is the fully qualified domain name (FQDN): machine name + domain. `ece387.local` is a private domain that exists only within your lab network — it is not registered on the public internet.

### 1.3 Install OpenLDAP

OpenLDAP is a directory service — think of it as a database that stores user accounts (usernames, UIDs, home directory paths, passwords). Every master computer will query this database to authenticate students.

```bash
# Update package list, then install:
#   slapd       — the OpenLDAP server daemon
#   ldap-utils  — command-line tools for querying/modifying the LDAP database
#   ldapscripts  — helper scripts for common tasks like adding users
sudo apt update && sudo apt upgrade -y
sudo apt install -y slapd ldap-utils ldapscripts
```

During install you are prompted for an admin password — set a strong one (e.g., `LdapAdmin2024!`). This is the LDAP admin password, not your Linux account password.

Now reconfigure the database to use your domain name. This step sets the "root" of the LDAP tree — everything hangs off `dc=ece387,dc=local`:

```bash
sudo dpkg-reconfigure slapd
```

Answer the prompts:

- Omit OpenLDAP server configuration? → **No**
- DNS domain name → `ece387.local`  *(this becomes dc=ece387,dc=local in LDAP)*
- Organization name → `ECE387`
- Admin password → `LdapAdmin2024!` (confirm)
- Remove database when slapd is purged? → **No**
- Move old database? → **Yes**

Verify the server is running and the base structure is reachable:

```bash
# Check that slapd is active
sudo systemctl status slapd

# Query the LDAP server — it should return the base entry with no errors.
# -x         = simple authentication (no Kerberos)
# -H         = server URI
# -b         = base DN to search from
# -D         = who you're connecting as (the admin account)
# -W         = prompt for password
ldapsearch -x -H ldap://localhost -b "dc=ece387,dc=local" -D "cn=admin,dc=ece387,dc=local" -W
```

### 1.4 Create LDAP Structure

LDAP organizes entries in a tree. Before adding users, you need to create the "folders" (called organizational units, or `ou`) that hold them. This is like creating directories before putting files inside.

```bash
# Write the structure definition to a file in LDIF format.
# LDIF (LDAP Data Interchange Format) is the text format used to describe
# entries you want to add, modify, or delete in the LDAP database.
cat > ~/Documents/ece387/structure.ldif << 'EOF'
# ou=students: holds all student user accounts
dn: ou=students,dc=ece387,dc=local
objectClass: organizationalUnit
ou: students

# ou=groups: holds group definitions
dn: ou=groups,dc=ece387,dc=local
objectClass: organizationalUnit
ou: groups

# ece387students: the group all students belong to.
# gidNumber 10000 is the shared group ID assigned to all students.
dn: cn=ece387students,ou=groups,dc=ece387,dc=local
objectClass: posixGroup
cn: ece387students
gidNumber: 10000
EOF

# Add these entries to the LDAP database
# -f = read from file instead of stdin
ldapadd -x -H ldap://localhost \
  -D "cn=admin,dc=ece387,dc=local" \
  -W -f ~/Documents/ece387/structure.ldif
```

### 1.5 Create a Course .bashrc Template

Before creating student accounts, set up the course environment template. Every student's `.bashrc` will be copied from this file so they all start with the correct ROS environment variables.

```bash
# Create the directory that will hold course-wide config files
sudo mkdir -p /etc/ece387

sudo nano /etc/ece387/bashrc_template
```

```bash
# ECE 387 Course Environment
# This file is sourced every time a terminal opens.

# ROS_DOMAIN_ID separates ROS2 traffic between different robot pairs.
# Each student should set this to a unique number matching their robot.
export ROS_DOMAIN_ID=30

export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source the ROS2 Jazzy environment so ros2 commands are available
source /opt/ros/jazzy/setup.bash

# Source the student's own workspace if it has been built
# The "2>/dev/null || true" suppresses the error if the workspace doesn't exist yet
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Shortcut to SSH into the robot.
# Students set ROBOT_IP in their own .bashrc, e.g.: export ROBOT_IP=10.42.0.1
alias robot='ssh ubuntu@$ROBOT_IP'
```

### 1.6 Create Student Accounts

Student IDs follow the pattern `a27-01` through `a27-30`. The script below:

1. Generates a hashed password for all accounts
2. Builds a single LDIF file with all 30 user entries
3. Creates each student's home directory on the server and populates it with the course `.bashrc`
4. Adds all accounts to LDAP in one command

```bash
cat > ~/Documents/ece387/create_students.sh << 'EOF'
#!/bin/bash
# Exit immediately if any command fails
set -e

# --- Configuration ---
LDAP_ADMIN_DN="cn=admin,dc=ece387,dc=local"
LDAP_ADMIN_PW="LdapAdmin2024!"    # your LDAP admin password
INITIAL_PW="TempPass2024!"        # students change this on first login
LDIF=~/Documents/ece387/students.ldif

# Generate a single password hash to reuse for all accounts.
# slappasswd hashes the password in a format OpenLDAP understands ({SSHA}...).
HASH=$(slappasswd -s "$INITIAL_PW")

# Clear the LDIF file if it already exists from a previous run
> $LDIF

# Loop from 01 to 30, creating one LDAP entry per student
for i in $(seq -w 1 30); do
  USERNAME="a27-${i}"

  # UID numbers must be unique integers. We use 10001–10030.
  # "10#$i" forces base-10 interpretation so "08" isn't treated as octal.
  UID_NUMBER=$((10000 + 10#$i))

  # Append this student's entry to the LDIF file.
  # Each entry is separated by a blank line (required by LDIF format).
  cat >> $LDIF << ENTRY
dn: uid=${USERNAME},ou=students,dc=ece387,dc=local
objectClass: inetOrgPerson
objectClass: posixAccount
objectClass: shadowAccount
uid: ${USERNAME}
cn: ${USERNAME}
sn: ${USERNAME}
uidNumber: ${UID_NUMBER}
gidNumber: 10000
homeDirectory: /home/students/${USERNAME}
loginShell: /bin/bash
userPassword: ${HASH}

ENTRY

  # Create the student's home directory on the NFS share.
  # /etc/skel contains default files (.bashrc, .profile, etc.) that are
  # copied into every new home directory.
  sudo mkdir -p /home/students/${USERNAME}
  sudo cp -r /etc/skel/. /home/students/${USERNAME}/

  # Overwrite .bashrc with the course template
  sudo cp /etc/ece387/bashrc_template /home/students/${USERNAME}/.bashrc

  # Set ownership to the student's UID and the shared group GID (10000).
  # chmod 700 means only the student can read/write their own directory.
  sudo chown -R ${UID_NUMBER}:10000 /home/students/${USERNAME}
  sudo chmod 700 /home/students/${USERNAME}

  echo "Prepared: ${USERNAME} (uid=${UID_NUMBER})"
done

# Add all 30 entries to LDAP in a single operation.
# -w = password (non-interactive, read from argument)
ldapadd -x -H ldap://localhost \
  -D "$LDAP_ADMIN_DN" \
  -w "$LDAP_ADMIN_PW" \
  -f $LDIF

echo ""
echo "Done. 30 student accounts created."
EOF

# Make the script executable, then run it
chmod +x ~/Documents/ece387/create_students.sh
bash ~/Documents/ece387/create_students.sh
```

Verify all accounts were created:

```bash
# Search the LDAP students OU and list just the uid field.
# You should see uid: a27-01 through uid: a27-30.
ldapsearch -x -H ldap://localhost \
  -b "ou=students,dc=ece387,dc=local" \
  -D "cn=admin,dc=ece387,dc=local" \
  -W uid uidNumber homeDirectory | grep "^uid:"
```

### 1.7 Configure NFS Home Directories

NFS (Network File System) lets the master computers mount the server's `/home/students` directory as if it were a local disk. When a student logs in to any master, their home directory appears from the server.

```bash
# Install the NFS server package
sudo apt install -y nfs-kernel-server

# Create the parent directory that holds all student home directories.
# Individual student dirs (a27-01, a27-02, ...) were already created in 1.6.
sudo mkdir -p /home/students
sudo chmod 755 /home/students
```

Edit the NFS exports file to define what is shared and who can access it:

```bash
sudo nano /etc/exports
```

```
# Share /home/students with all machines on the 192.168.1.0/24 subnet.
# rw             = read and write access
# sync           = write data to disk before acknowledging — safer than async
# no_subtree_check = disables subtree checking, improves reliability
# no_root_squash = allows root on clients to act as root here (needed for
#                  creating/chowning files during setup)
/home/students  192.168.1.0/24(rw,sync,no_subtree_check,no_root_squash)
```

```bash
# Re-read /etc/exports and apply the new export
sudo exportfs -rav

# Enable and start the NFS server
sudo systemctl enable --now nfs-server

# Verify the export is listed — should show /home/students
showmount -e localhost
```

### 1.8 Open Required Firewall Ports

```bash
# Allow SSH so you can manage the server remotely
sudo ufw allow OpenSSH

# Allow LDAP queries from master computers (port 389)
sudo ufw allow 389/tcp

# Allow NFS traffic (port 2049)
sudo ufw allow 2049/tcp

# Allow all traffic from the lab subnet (covers NFS port-mapper and other NFS ports)
sudo ufw allow from 192.168.1.0/24

# Enable the firewall
sudo ufw enable

# Verify the rules
sudo ufw status
```

---

## Part 2: Master Computer Setup (NUC 9 × 14)

Do this on each of the 14 master computers. Most steps can be scripted and run via Ansible to configure all 14 machines simultaneously — see Part 5.

### 2.1 Fresh Install Ubuntu 24.04 Desktop

1. Boot from Ubuntu 24.04 Desktop ISO.
2. During install:
   - Hostname: `master01`, `master02`, ... `master14`
   - Create local admin account: `ece387admin`
3. After install, set a static IP on the lab Ethernet interface so the machine can always reach the login server:

```bash
# Find your ethernet interface name
ip a

sudo nano /etc/netplan/01-lab-network.yaml
```

```yaml
network:
  version: 2
  ethernets:
    enp86s0:                          # your ethernet interface
      dhcp4: false
      addresses: [192.168.1.101/24]   # increment per machine: .102, .103, etc.
      routes:
        - to: default
          via: 192.168.1.1            # lab gateway
      nameservers:
        addresses: [8.8.8.8]
```

```bash
sudo netplan apply

# Confirm the master can reach the login server before proceeding
ping 192.168.1.10
```

### 2.2 Install ROS2 Jazzy

```bash
# Install prerequisites for adding a new apt repository
sudo apt install -y software-properties-common curl

# Download and store the ROS2 package signing key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the ROS2 apt repository for Ubuntu Noble (24.04)
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update

# Install ROS2 Jazzy Desktop (includes rviz2, rqt, etc.) plus TurtleBot3 packages
sudo apt install -y ros-jazzy-desktop ros-jazzy-turtlebot3* ros-dev-tools
```

sudo apt install -y ros-humble-gazebo-*
sudo apt install -y ros-humble-usb-cam ros-humble-image-proc
sudo apt install -y ros-humble-camera-calibration
sudo apt install -y ros-humble-apriltag ros-humble-apriltag-ros
sudo apt install -y tree
sudo apt install -y jstest-gtk  

alias bringup='ssh pi@robotX '\''ros2 launch turtlebot3_bringup robot.launch.py'\'

# Function to build with optional arguments

function ccbuild() {
    cd ~/master_ws && colcon build --symlink-install "$@"
    source ~/master_ws/install/setup.bash
}

# Export the function to make it available in the shell

export -f ccbuild

source /opt/ros/jazzy/setup.bash
source ~/master_ws/install/setup.bash
export ROS_DOMAIN_ID=99  # For master0 and robot0

export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-02 # replace with LDS-02 if using new LIDAR

source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /usr/share/gazebo/setup.sh
source /usr/share/colcon_cd/function/colcon_cd.sh
export_colcon_cd_root=/opt/ros/jazzy/

### 2.3 Configure SSSD for LDAP Authentication

SSSD (System Security Services Daemon) is the bridge between the master computer and the login server's LDAP database. When a student types their username and password, SSSD queries the LDAP server to verify their identity.

```bash
# Install SSSD and its LDAP backend, plus PAM/NSS libraries that let Linux
# use SSSD for login (PAM) and user lookups like "id username" (NSS)
sudo apt install -y sssd sssd-ldap libpam-sss libnss-sss oddjob oddjob-mkhomedir
```

Create the SSSD configuration file:

```bash
sudo nano /etc/sssd/sssd.conf
```

```ini
[sssd]
# Services to run:
#   nss = name service (allows "id username", "getent passwd", etc.)
#   pam = pluggable authentication module (handles login)
services = nss, pam
domains = ece387.local
config_file_version = 2

[domain/ece387.local]
# id_provider = where to look up user information (UID, home dir, etc.)
# auth_provider = where to verify passwords
id_provider = ldap
auth_provider = ldap

# URI of the LDAP server (the login server's IP)
ldap_uri = ldap://192.168.1.10

# Base DN: the root of the LDAP tree to search
ldap_search_base = dc=ece387,dc=local

# Narrow searches to the students OU for efficiency
ldap_user_search_base = ou=students,dc=ece387,dc=local
ldap_group_search_base = ou=groups,dc=ece387,dc=local

# Credentials SSSD uses to bind (connect) to the LDAP server for lookups
ldap_default_bind_dn = cn=admin,dc=ece387,dc=local
ldap_default_authtok_type = password
ldap_default_authtok = LdapAdmin2024!

# Do not use TLS for now (simpler setup for a local lab network)
ldap_id_use_start_tls = false

# Cache credentials locally so students can still log in if the server
# is temporarily unreachable (offline mode)
cache_credentials = true

# Pre-load all user accounts so "getent passwd" and tab-completion work
enumerate = true
```

```bash
# sssd.conf must not be readable by other users — SSSD refuses to start otherwise
sudo chmod 600 /etc/sssd/sssd.conf

# Enable and start SSSD
sudo systemctl enable --now sssd

# Tell PAM to create a home directory on first login if one doesn't exist locally
sudo pam-auth-update --enable mkhomedir

# Test: this should return the student's uid, gid, and home directory path
id a27-01
```

### 2.4 Mount NFS Home Directories

autofs automatically mounts a network directory the moment it is accessed, and unmounts it after a period of inactivity. This is more efficient than a static mount in `/etc/fstab` because the mount only happens when needed.

```bash
# Install autofs (automounter) and nfs-common (NFS client tools)
sudo apt install -y autofs nfs-common
```

Tell autofs to manage the `/home/students` mount point:

```bash
sudo nano /etc/auto.master.d/students.autofs
```

```
# When anything under /home/students is accessed, use the rules in /etc/auto.students.
# --timeout=600 means unmount after 10 minutes of inactivity.
/home/students  /etc/auto.students  --timeout=600
```

Define the NFS mount rule:

```bash
sudo nano /etc/auto.students
```

```
# The * wildcard matches any username.
# When /home/students/a27-01 is accessed, autofs mounts:
#   192.168.1.10:/home/students/a27-01
# The & at the end substitutes the matched username.
#
# soft     = give up if the server is unreachable rather than hanging forever
# timeo=30 = wait 3 seconds per retry attempt (units are 0.1s)
# retrans=2 = retry twice before giving up
*  -fstype=nfs,soft,timeo=30,retrans=2  192.168.1.10:/home/students/&
```

```bash
# Enable autofs at boot and start it now
sudo systemctl enable --now autofs
sudo systemctl restart autofs

# Test: switch to a student account — their home directory should mount automatically
sudo su - a27-01
pwd          # should show /home/students/a27-01
ls -la       # should show .bashrc, .profile, etc.
exit
```

### 2.5 Grant Students sudo Access (Optional)

If students need `sudo` privileges (e.g., to install ROS packages during labs):

```bash
sudo nano /etc/sudoers.d/ece387-students
```

```
# Members of the ece387students group can run any command as any user.
# The % prefix means this applies to a group, not an individual user.
%ece387students  ALL=(ALL) ALL
```

```bash
# sudoers files must have exactly these permissions — sudo ignores files with wrong perms
sudo chmod 440 /etc/sudoers.d/ece387-students
```

### 2.6 Configure WiFi Interfaces

Each master has two WiFi adapters. Use NetworkManager (`nmcli`) to configure them:

```bash
# List all network interfaces and their current state
nmcli device status

# Connect the external USB WiFi dongle to the internet network.
# Replace wlx<mac_of_dongle> with the actual interface name (e.g., wlx00e04c360001)
nmcli dev wifi connect "AFAcademy_Guest" ifname wlx<mac_of_dongle>

# Make this connection reconnect automatically after every reboot
nmcli connection modify "AFAcademy_Guest" connection.autoconnect yes

# The internal WiFi adapter is left for students to connect to their robot's
# access point during lab — they configure this per-robot.
```

---

## Part 3: Resilience — Server Down or Network Unreliable

Two failure modes and how each is handled:

**Authentication (SSSD):** Already resilient. `cache_credentials = true` in `sssd.conf` means after a student logs in once, SSSD saves their credentials in a local encrypted cache. If the login server goes down, they can still log in to any master they have previously used.

**Home directories (NFS):** The more serious problem — if the NFS server is unreachable, the student's home directory cannot be mounted, so they lose access to `.bashrc`, SSH keys, and their ROS workspace.

The solution combines two approaches:

### Option 1: GitHub as the Safety Net

Require students to keep their ROS workspace in a GitHub repository as part of the course workflow. If NFS is unreachable, a student can recover in minutes:

```bash
# Clone from GitHub using HTTPS (no SSH key needed)
git clone https://github.com/<username>/<repo>.git ~/ros2_ws
cd ~/ros2_ws && colcon build
```

AFAcademy_Guest WiFi provides internet access independently of the ECE lab network, so GitHub is reachable even when the login server is not.

### Option 2: Soft NFS Mounts

Without `soft`, a failed NFS mount causes the accessing process to hang indefinitely — the terminal freezes and the student cannot do anything. `soft` mounts fail fast instead, so the student gets an error message rather than a frozen session.

This is already included in the `/etc/auto.students` configuration in Section 2.4:

```
*  -fstype=nfs,soft,timeo=30,retrans=2  192.168.1.10:/home/students/&
```

If you need to change this after initial setup, edit the file and restart autofs:

```bash
sudo systemctl restart autofs
```

---

## Part 4: Student Workflow

When a student moves to a different station:

1. Log in with their LDAP username (e.g., `a27-01`) and password — SSSD authenticates against the login server and their home directory mounts automatically via NFS.
2. `.bashrc`, `.ssh/`, and workspace files are immediately available — no re-setup needed.
3. Connect to their robot:

   ```bash
   # Connect the internal WiFi adapter to the robot's access point
   nmcli dev wifi connect "robot_XX_ap" password "robotpassword" ifname wlan0

   # SSH into the robot
   ssh ubuntu@10.42.0.1
   ```

4. Their GitHub SSH key is already in `~/.ssh/` — `git push` and `git pull` work immediately.

---

## Part 5: Ansible Automation (Recommended for 14 Machines)

Rather than repeating Part 2 on each master by hand, Ansible lets you run the same commands on all 14 machines simultaneously from the login server.

```bash
# Install Ansible on the login server
sudo apt install -y ansible

# Create an inventory file listing all master computers
# Ansible reads this to know which machines to configure
cat > ~/Documents/ece387/masters-inventory.ini << 'EOF'
[masters]
master01 ansible_host=192.168.1.101
master02 ansible_host=192.168.1.102
master03 ansible_host=192.168.1.103
master04 ansible_host=192.168.1.104
master05 ansible_host=192.168.1.105
master06 ansible_host=192.168.1.106
master07 ansible_host=192.168.1.107
master08 ansible_host=192.168.1.108
master09 ansible_host=192.168.1.109
master10 ansible_host=192.168.1.110
master11 ansible_host=192.168.1.111
master12 ansible_host=192.168.1.112
master13 ansible_host=192.168.1.113
master14 ansible_host=192.168.1.114

[masters:vars]
ansible_user=ece387admin
ansible_become=yes
EOF

# Run a playbook (write the playbook separately covering sections 2.2–2.6)
ansible-playbook -i ~/Documents/ece387/masters-inventory.ini setup-masters.yml
```

---

## Part 6: Maintenance

### Add a new student

```bash
# Step 1: Generate a hashed password for the new student
slappasswd -s NewStudentPass!
# Copy the {SSHA}... output

# Step 2: Add the account to LDAP
# Use uidNumber 10031 (or the next available number after your existing students)
ldapadd -x -H ldap://localhost \
  -D "cn=admin,dc=ece387,dc=local" -W << 'EOF'
dn: uid=a27-31,ou=students,dc=ece387,dc=local
objectClass: inetOrgPerson
objectClass: posixAccount
objectClass: shadowAccount
uid: a27-31
cn: a27-31
sn: a27-31
uidNumber: 10031
gidNumber: 10000
homeDirectory: /home/students/a27-31
loginShell: /bin/bash
userPassword: {SSHA}PASTE_HASH_HERE
EOF

# Step 3: Create the home directory on the server
sudo mkdir -p /home/students/a27-31
sudo cp -r /etc/skel/. /home/students/a27-31/
sudo cp /etc/ece387/bashrc_template /home/students/a27-31/.bashrc
sudo chown -R 10031:10000 /home/students/a27-31
sudo chmod 700 /home/students/a27-31
```

### Reset a student password

```bash
# Replace a27-01 with the student's username, and set the new password with -s
ldappasswd -H ldap://localhost \
  -D "cn=admin,dc=ece387,dc=local" \
  -W -s NewPassword! \
  "uid=a27-01,ou=students,dc=ece387,dc=local"
```

### List all student accounts

```bash
ldapsearch -x -H ldap://localhost \
  -b "ou=students,dc=ece387,dc=local" \
  -D "cn=admin,dc=ece387,dc=local" \
  -W uid cn
```

### Check who is logged in across all masters

```bash
# SSH into each master and run 'who' to see logged-in users
for i in $(seq -w 1 14); do
  echo "=== master$i ==="
  ssh ece387admin@192.168.1.1${i} who 2>/dev/null
done
```

### Back up home directories and LDAP database

```bash
# Sync all student home directories to a backup location
sudo rsync -av /home/students/ /backup/students/

# Export the entire LDAP database to a text file (LDIF format)
# This file can be used to restore the database on a new server
sudo slapcat > ~/Documents/ece387/ldap-backup-$(date +%Y%m%d).ldif
```

---

## Troubleshooting

| Problem | Command | Fix |
|---------|---------|-----|
| Student can't log in | `ldapsearch -x -H ldap://localhost -b "ou=students,dc=ece387,dc=local" -D "cn=admin,dc=ece387,dc=local" -W "(uid=a27-01)"` | Check account exists; verify password hash is valid |
| Home dir not mounting | `automount -v` | Check NFS exports on server, autofs config on master, server reachable |
| SSSD not resolving users | `sssctl user-checks a27-01` | Restart sssd: `systemctl restart sssd` |
| LDAP connection refused | `ldapsearch -x -H ldap://192.168.1.10 -b "dc=ece387,dc=local" -x` | Check `systemctl status slapd` on server |
| NFS mount fails | `showmount -e 192.168.1.10` | Check firewall on server; verify `nfs-server` is running |
| Login hangs (no soft mount) | `journalctl -u autofs` | Ensure `soft,timeo=30,retrans=2` is in `/etc/auto.students` |
| Login slow (~30s) | `journalctl -u sssd` | Check LDAP URI is reachable: `ping 192.168.1.10` |
