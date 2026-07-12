# ECE 387 Login Server Setup

This guide covers setting up the **login server** — the machine that hosts OpenLDAP (user accounts) and NFS (shared home directories) for the lab. It is one of three companion guides:

- **login-server.md** (this file) — the login server itself
- [login-client.md](login-client.md) — master computers that authenticate against the login server
- [master-standalone-setup.md](master-standalone-setup.md) — a single master with a local account, no login server

## Why a Login Server?

A login server (OpenLDAP + SSSD + NFS) is only worth the setup effort in one specific situation: **a shared lab of fixed, dedicated computers that many different students rotate through.**

**When a login server helps:**
- The lab has a bank of master computers (e.g., 14 NUCs) and students do not own or sit at the same machine every session.
- A student should be able to sit down at *any* master, log in with their own credentials, and immediately have their `.bashrc`, SSH keys, and ROS workspace available — without re-cloning or re-configuring anything.
- You want to manage 20–30 student accounts centrally: create them once, reset a forgotten password in one command, and push a `.bashrc` update to every account at once (see [Maintenance](#3-maintenance)).
- You need some isolation between students who share the same physical hardware (restricted sudo, separate home directories, no ability to read another student's files).

**When a login server is overkill:**
- Students each have their own laptop or a personal workstation they always use. In that case there is nothing to "roam" between — a local Ubuntu account already gives each student a persistent home directory, and centralized authentication just adds an extra server to install, patch, and keep running (plus a single point of failure if it goes down). Use [master-standalone-setup.md](master-standalone-setup.md) instead — it sets up the same ROS 2 environment with a normal local account and no LDAP/NFS dependency.
- You only have a handful of machines and can simply assign one student (or a fixed pair) per machine for the term.

**The trade-off:** a login server buys you station-independence and central account management, at the cost of standing up and maintaining an extra server, plus a dependency on the network (mitigated in [login-client.md](login-client.md) with SSSD credential caching and soft NFS mounts). For ECE 387's AFA lab — a fixed set of 14 dedicated NUCs used by a rotating roster of ~30 students across sections — that trade-off is worth it.

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

**Key concept:** Student home directories live on the login server and are NFS-mounted on every master. When a student logs in to any master (set up per [login-client.md](login-client.md)), their home directory (`.bashrc`, SSH keys, ROS workspace) is automatically available — no per-machine setup needed.

> **Why not FreeIPA?** `freeipa-server` is not packaged for Ubuntu 24.04. OpenLDAP + SSSD is the standard replacement and provides the same centralized login and shared home directory functionality.

---

## Network Planning

Assign static IPs on your lab's Ethernet network before starting. All machines must be on the same subnet so they can reach each other.

| Host | Hostname | IP |
|------|----------|----|
| Login Server | `ece387server` | `192.168.0.151` |
| Master 01 | `master01` | `192.168.0.101` |
| Master 02 | `master02` | `192.168.0.102` |
| ... | ... | ... |
| Master 14 | `master14` | `192.168.0.114` |

> The login server IP (`192.168.0.151`) reflects the home test setup. In the AFA lab, adjust all IPs to match the actual lab subnet. This guide uses the domain `ece387.local`.

---

## 1. Login Server Setup

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
      addresses: [192.168.0.151/24]
      routes:
        - to: default
          via: 192.168.0.1        # your router IP (home: 192.168.0.1, lab: adjust accordingly)
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
192.168.0.151  ece387server.ece387.local  ece387server
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

During install you are prompted for an admin password — set a strong one (e.g., `LdapAdmin387!`). This is the LDAP admin password, not your Linux account password.

Now reconfigure the database to use your domain name. This step sets the "root" of the LDAP tree — everything hangs off `dc=ece387,dc=local`:

```bash
sudo dpkg-reconfigure slapd
```

Answer the prompts:
- Omit OpenLDAP server configuration? → **No**
- DNS domain name → `ece387.local`  *(this becomes dc=ece387,dc=local in LDAP)*
- Organization name → `ECE387`
- Admin password → `LdapAdmin387!` (confirm)
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

### 1.5 Create Course Config Templates

Before creating student accounts, set up the course templates. These files are copied into every student's home directory and can be updated and re-pushed at any time.

```bash
# Create the directory that will hold course-wide config files
sudo mkdir -p /etc/ece387
```

#### .bashrc Template

The `.bashrc` is sourced every time a terminal opens. It sets ROS environment variables and loads the student's personal customizations from a separate file, so you can update course settings without overwriting students' own additions.

```bash
sudo nano /etc/ece387/bashrc_template
```

```bash
# ECE 387 Course Environment
# This file is managed by the instructor — do not edit directly.
# Add your own customizations to ~/.bashrc_personal instead.

# ROS_DOMAIN_ID separates ROS2 traffic between different robot pairs.
# Each student should set this in ~/.bashrc_personal, e.g.:
#   export ROS_DOMAIN_ID=15
export ROS_DOMAIN_ID=30

export TURTLEBOT3_MODEL=burger
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source the ROS2 Jazzy environment so ros2 commands are available
source /opt/ros/jazzy/setup.bash

# Source the student's own workspace if it has been built
# The "2>/dev/null || true" suppresses the error if the workspace doesn't exist yet
source ~/ros2_ws/install/setup.bash 2>/dev/null || true

# Shortcut to SSH into the robot.
# Students set ROBOT_IP in ~/.bashrc_personal, e.g.: export ROBOT_IP=10.42.0.1
alias robot='ssh ubuntu@$ROBOT_IP'

# Load student's personal customizations (robot IP, aliases, etc.)
# This file is never overwritten by the instructor.
source ~/.bashrc_personal 2>/dev/null || true
```

#### .inputrc Template

`.inputrc` configures readline behavior — used by bash for history search and tab completion.

```bash
sudo nano /etc/ece387/inputrc_template
```

```
# Arrow keys search history based on what's already typed
"\e[A": history-search-backward
"\e[B": history-search-forward

# Case-insensitive tab completion
set completion-ignore-case on
```

#### Pushing Updates to All Students

The templates are only copied automatically when accounts are first created. To push changes to all existing students at any time:

```bash
for user in /home/students/a27-*/; do
  sudo cp /etc/ece387/bashrc_template "${user}.bashrc"
  sudo cp /etc/ece387/inputrc_template "${user}.inputrc"
  echo "Updated: $user"
done
```

> This overwrites `.bashrc` and `.inputrc` but leaves `.bashrc_personal` untouched, so student customizations are preserved.

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
LDAP_ADMIN_PW="LdapAdmin387!"    # your LDAP admin password
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

  # Copy course config templates
  sudo cp /etc/ece387/bashrc_template /home/students/${USERNAME}/.bashrc
  sudo cp /etc/ece387/inputrc_template /home/students/${USERNAME}/.inputrc

  # Create an empty .bashrc_personal for student customizations (robot IP, aliases, etc.)
  # This file is sourced at the end of .bashrc and is never overwritten by the instructor.
  sudo touch /home/students/${USERNAME}/.bashrc_personal

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
# Share /home/students with all machines on the 192.168.0.0/24 subnet.
# rw             = read and write access
# sync           = write data to disk before acknowledging — safer than async
# no_subtree_check = disables subtree checking, improves reliability
# no_root_squash = allows root on clients to act as root here (needed for
#                  creating/chowning files during setup)
/home/students  192.168.0.0/24(rw,sync,no_subtree_check,no_root_squash)
# Note: adjust subnet to match your lab network (e.g., 192.168.0.0/24 for home testing)
```

```bash
# Re-read /etc/exports and apply the new export
sudo exportfs -rav

# Enable and start the NFS server
sudo systemctl enable --now nfs-server

# Verify the export is listed — should show /home/students
showmount -e localhost
```

> `no_root_squash` is used here to simplify initial account creation. Once accounts are set up, switch to `root_squash` for better isolation between students — see "Grant Students Restricted sudo Access" in [login-client.md](login-client.md).

### 1.8 Open Required Firewall Ports

```bash
# Allow SSH so you can manage the server remotely
sudo ufw allow OpenSSH

# Allow LDAP queries from master computers (port 389)
sudo ufw allow 389/tcp

# Allow NFS traffic (port 2049)
sudo ufw allow 2049/tcp

# Allow rpcbind/portmapper (port 111) — required for showmount and NFS negotiation
sudo ufw allow 111/tcp
sudo ufw allow 111/udp

# Allow all traffic from the lab subnet
sudo ufw allow from 192.168.0.0/24

# Enable the firewall
sudo ufw enable

# Verify the rules
sudo ufw status
```

Once the server is up, continue to [login-client.md](login-client.md) to configure each master to authenticate against it.

---

## 2. Resilience — Server Down or Network Unreliable

**Authentication (SSSD):** Handled on the client side — after a student logs in once, SSSD on each master caches their credentials locally so they can still log in if the login server goes down. See [login-client.md](login-client.md).

**Home directories (NFS):** If the NFS server is unreachable, students lose access to `.bashrc`, SSH keys, and their ROS workspace on any master. Mitigations (GitHub as a safety net, soft NFS mounts) are configured on the client side — see [login-client.md](login-client.md).

---

## 3. Maintenance

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
sudo cp /etc/ece387/inputrc_template /home/students/a27-31/.inputrc
sudo touch /home/students/a27-31/.bashrc_personal
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
| LDAP connection refused | `ldapsearch -x -H ldap://192.168.0.151 -b "dc=ece387,dc=local" -x` | Check `systemctl status slapd` on server |
| NFS mount fails (from a client) | `showmount -e 192.168.0.151` | Check firewall on server; verify `nfs-server` is running |

For client-side symptoms (home directory not mounting, SSSD not resolving users, login hangs or is slow), see the troubleshooting table in [login-client.md](login-client.md).
