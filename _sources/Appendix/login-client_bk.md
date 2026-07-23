# ECE 387 Login Client Setup (Master Computers)

This guide covers setting up a **master computer as a login client** — a machine that authenticates students against the login server (via SSSD) and mounts their home directory over NFS, so any student can sit at any master. It is one of three companion guides:

- [login-server.md](login-server.md) — the login server itself
- **login-client.md** (this file) — master computers that authenticate against the login server
- [master-standalone-setup.md](master-standalone-setup.md) — a single master with a local account, no login server

Complete [login-server.md](login-server.md) first — the login server must be running before a client can authenticate against it.

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

## 1. Master Computer Setup (NUC 9 × 14)

Do this on each of the 14 master computers. Most steps can be scripted and run via Ansible to configure all 14 machines simultaneously — see [Ansible Automation](#3-ansible-automation-recommended-for-14-machines).

### 1.1 Fresh Install Ubuntu 24.04 Desktop

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
      addresses: [192.168.0.101/24]   # increment per machine: .102, .103, etc.
      routes:
        - to: default
          via: 192.168.0.1            # lab gateway
      nameservers:
        addresses: [8.8.8.8]
```

```bash
sudo netplan apply

# Confirm the master can reach the login server before proceeding
ping 192.168.0.151
```

### 1.2 Install ROS2 Jazzy

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

### 1.3 Configure SSSD for LDAP Authentication

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
ldap_uri = ldap://192.168.0.151

# Base DN: the root of the LDAP tree to search
ldap_search_base = dc=ece387,dc=local

# Narrow searches to the students OU for efficiency
ldap_user_search_base = ou=students,dc=ece387,dc=local
ldap_group_search_base = ou=groups,dc=ece387,dc=local

# Credentials SSSD uses to bind (connect) to the LDAP server for lookups
ldap_default_bind_dn = cn=admin,dc=ece387,dc=local
ldap_default_authtok_type = password
ldap_default_authtok = LdapAdmin387!

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

# SSSD starts up before the LDAP connection is fully established, so it may cache
# "offline" (failed) responses during those first few seconds. Clear the cache
# to force fresh lookups now that the backend is online.
sudo systemctl stop sssd
sudo rm -rf /var/lib/sss/db/*
sudo systemctl start sssd
sleep 5

# Test: this should return the student's uid, gid, and home directory path
# Expected: uid=10001(a27-01) gid=10000(ece387students) groups=10000(ece387students)
id a27-01
```

### 1.4 Mount NFS Home Directories

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
#   192.168.0.151:/home/students/a27-01
# The & at the end substitutes the matched username.
#
# soft     = give up if the server is unreachable rather than hanging forever
# timeo=30 = wait 3 seconds per retry attempt (units are 0.1s)
# retrans=2 = retry twice before giving up
*  -fstype=nfs,soft,timeo=30,retrans=2  192.168.0.151:/home/students/&
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

### 1.5 Grant Students Restricted sudo Access

Students need `sudo` for package management and system commands during ROS labs, but should not be able to access other students' home directories via sudo.

Two layers of protection are used together:
- **`root_squash` on the NFS server** — maps root on the master to an anonymous unprivileged user on the server, so `sudo cat /home/students/a27-02/file` is rejected by the server even if the student tries it
- **Command whitelist in sudoers** — restricts which commands can be run with sudo at all

First, enable `root_squash` on the **login server**:

```bash
sudo nano /etc/exports
```

Change `no_root_squash` to `root_squash`:
```
/home/students  192.168.0.0/24(rw,sync,no_subtree_check,root_squash)
```

```bash
sudo exportfs -rav
```

Then on each **master**, create the sudoers whitelist:

```bash
sudo nano /etc/sudoers.d/ece387-students
```

```
# ECE387 student sudo permissions
# The % prefix applies the rule to a group rather than an individual user.
# NOPASSWD means students are not prompted for a password — practical in a lab.

# Package management — installing ROS packages and dependencies
Cmnd_Alias ECE387_PKG = /usr/bin/apt, /usr/bin/apt-get, /usr/bin/dpkg, \
                        /usr/bin/pip, /usr/bin/pip3

# Network commands — configuring WiFi and checking connections
Cmnd_Alias ECE387_NET = /usr/bin/nmcli, /usr/sbin/ip

# ROS-specific tools
# Use "which colcon" and "which rosdep" to confirm paths on your system
Cmnd_Alias ECE387_ROS = /usr/bin/rosdep, /usr/bin/colcon

# System services — starting/stopping ROS or robot-related services
Cmnd_Alias ECE387_SVC = /usr/bin/systemctl

# System power — Lab1 requires sudo shutdown and sudo reboot on the master
Cmnd_Alias ECE387_PWR = /usr/sbin/shutdown, /usr/sbin/reboot

%ece387students ALL=(ALL) NOPASSWD: ECE387_PKG, ECE387_NET, ECE387_ROS, ECE387_SVC, ECE387_PWR
```

```bash
# sudoers files must have exactly 440 permissions — sudo silently ignores files with wrong perms
sudo chmod 440 /etc/sudoers.d/ece387-students
```

> **Note:** `ssh`, `cat`, and `nano` are intentionally excluded from the sudo whitelist — students don't need sudo to run them. They can SSH to their robots, read, and edit files freely without sudo. Excluding them from sudo prevents `sudo cat /home/students/a27-02/file` style attacks, and `root_squash` on the server provides a second layer of protection.

To add more commands to the whitelist later, find the full path first:
```bash
which <command>   # e.g., which colcon → /usr/bin/colcon
```
Then add the path to the appropriate `Cmnd_Alias` line.

### 1.6 Configure WiFi Interfaces

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

## 2. Resilience — Server Down or Network Unreliable

Two failure modes and how each is handled:

**Authentication (SSSD):** Already resilient. `cache_credentials = true` in `sssd.conf` (Section 1.3) means after a student logs in once, SSSD saves their credentials in a local encrypted cache. If the login server goes down, they can still log in to any master they have previously used.

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

This is already included in the `/etc/auto.students` configuration in Section 1.4:

```
*  -fstype=nfs,soft,timeo=30,retrans=2  192.168.0.151:/home/students/&
```

If you need to change this after initial setup, edit the file and restart autofs:

```bash
sudo systemctl restart autofs
```

---

## 3. Student Workflow

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

## 4. Ansible Automation (Recommended for 14 Machines)

Rather than repeating Section 1 on each master by hand, Ansible lets you run the same commands on all 14 machines simultaneously from the login server.

```bash
# Install Ansible on the login server
sudo apt install -y ansible

# Create an inventory file listing all master computers
# Ansible reads this to know which machines to configure
cat > ~/Documents/ece387/masters-inventory.ini << 'EOF'
[masters]
master01 ansible_host=192.168.0.101
master02 ansible_host=192.168.0.102
master03 ansible_host=192.168.0.103
master04 ansible_host=192.168.0.104
master05 ansible_host=192.168.0.105
master06 ansible_host=192.168.0.106
master07 ansible_host=192.168.0.107
master08 ansible_host=192.168.0.108
master09 ansible_host=192.168.0.109
master10 ansible_host=192.168.0.110
master11 ansible_host=192.168.0.111
master12 ansible_host=192.168.0.112
master13 ansible_host=192.168.0.113
master14 ansible_host=192.168.0.114

[masters:vars]
ansible_user=ece387admin
ansible_become=yes
EOF

# Run a playbook (write the playbook separately covering sections 1.2–1.6)
ansible-playbook -i ~/Documents/ece387/masters-inventory.ini setup-masters.yml
```

---

## 5. Maintenance

### Check who is logged in across all masters

```bash
# SSH into each master and run 'who' to see logged-in users
for i in $(seq -w 1 14); do
  echo "=== master$i ==="
  ssh ece387admin@192.168.1.1$(printf '%02d' $((10#$i))) who 2>/dev/null
done
```

---

## Troubleshooting

| Problem | Command | Fix |
|---------|---------|-----|
| Home dir not mounting | `automount -v` | Check NFS exports on server, autofs config on master, server reachable |
| SSSD not resolving users | `sssctl user-checks a27-01` | Restart sssd: `systemctl restart sssd` |
| NFS mount fails | `showmount -e 192.168.0.151` | Check firewall on server; verify `nfs-server` is running |
| Login hangs (no soft mount) | `journalctl -u autofs` | Ensure `soft,timeo=30,retrans=2` is in `/etc/auto.students` |
| Login slow (~30s) | `journalctl -u sssd` | Check LDAP URI is reachable: `ping 192.168.0.151` |

For server-side symptoms (student account issues, LDAP connection refused), see the troubleshooting table in [login-server.md](login-server.md).
