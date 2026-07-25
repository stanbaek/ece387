# Master (Client) Setup

This guide covers setting up a **master computer as a login client** — a machine that authenticates students against the login server (via SSSD) and mounts their home directory over NFS, so any student can sit at any master. It is one of three companion guides:

- [Login Server Setup](login-server.md) — the login server itself
- [Master (Client) Setup](login-client.md) — master computers that authenticate against the login server
- [Master Setup - Standalone](MasterSetupJazzy.md) — a single master with a local account, no login server

Complete [login-server.md](login-server.md) first — the login server must be running before a client can authenticate against it.

---

## Network Planning

Only the login server needs a static IP. Master computers get a **dynamic IP** from the lab's WiFi router, which acts as the DHCP server.

| Host | Hostname | IP |
|------|----------|----|
| WiFi router (DHCP server / gateway) | — | `10.99.1.1` (SSID `ECE387`) |
| Login Server | `ece387server` | `10.99.1.50` (static) |
| Master 01–14 | `master01`–`master14` | DHCP-assigned (dynamic) |

> Masters are identified by hostname, not IP, since their IP can change after a reboot or DHCP lease renewal. Ubuntu Desktop ships with `avahi-daemon` (mDNS) enabled by default, so each master is reachable at `<hostname>.local` (e.g. `master01.local`) from any other machine on the `ECE387` network, regardless of its current DHCP-assigned IP. This guide uses the domain `ece387.local` for LDAP.

---

## 1. Master Computer Setup (NUC 9 × 14)

Do this on each of the 14 master computers. Most steps can be scripted and run via Ansible to configure all 14 machines simultaneously — see [Ansible Automation](#4-ansible-automation-recommended-for-14-machines).

### 1.1 Fresh Install Ubuntu 24.04 Desktop

1. Boot from Ubuntu 24.04 Desktop ISO.
2. During install:
   - Hostname: `master01`, `master02`, ... `master14`
   - Create local admin account: `ece387admin`
   - Connect to the `ECE387` WiFi network (or Ethernet, if wired) during setup — Ubuntu will request a **dynamic IP via DHCP by default**, which is what we want. No static IP configuration is needed on the masters.
3. After install, confirm the master got an address from the router and can reach the login server:

```bash
# Confirm a DHCP lease was obtained (look for an inet addr on your WiFi/ethernet interface)
ip a

# Confirm avahi-daemon (mDNS) is installed and running — this is what lets other
# machines find this master at masterNN.local even though its IP can change
systemctl status avahi-daemon

# Confirm the master can reach the login server before proceeding
ping 10.99.1.50
```

> If `avahi-daemon` isn't running: `sudo apt install -y avahi-daemon && sudo systemctl enable --now avahi-daemon`.

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
ldap_uri = ldap://10.99.1.50

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
# Expected: uid=20000(a27-m0) gid=10000(ece387students) groups=10000(ece387students)
id a27-m0
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
# When /home/students/a27-m0 is accessed, autofs mounts:
#   10.99.1.50:/home/students/a27-m0
# The & at the end substitutes the matched username.
#
# soft     = give up if the server is unreachable rather than hanging forever
# timeo=30 = wait 3 seconds per retry attempt (units are 0.1s)
# retrans=2 = retry twice before giving up
*  -fstype=nfs,soft,timeo=30,retrans=2  10.99.1.50:/home/students/&
```

```bash
# Enable autofs at boot and start it now
sudo systemctl enable --now autofs
sudo systemctl restart autofs

# Test: switch to a student account — their home directory should mount automatically
sudo su - a27-m0
pwd          # should show /home/students/a27-m0
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
/home/students  10.99.1.0/24(rw,sync,no_subtree_check,root_squash)
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

Each master has two WiFi adapters:

- **Built-in adapter** (`wlo1`) — left unconfigured here; students point this at their robot's access point per-lab via `nmcli`.
- **External USB dongle** — connects to the lab/internet networks (`ECE387`, `AF_ACADEMY_GUEST`, `ECE`). Since each machine's dongle has a different MAC address, we rename it to a consistent interface name (`wlan1`) so the same config works on all 14 masters.

**a. Rename any USB WiFi dongle to `wlan1`** (matches by bus/type, not MAC, so it works regardless of which dongle is plugged into a given machine):

```bash
sudo nano /etc/systemd/network/10-usb-wifi.link
```

```ini
[Match]
Type=wlan
Property=ID_BUS=usb

[Link]
Name=wlan1
```

```bash
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=net --action=add
# Unplug and replug the dongle (or reboot) so it re-enumerates under the new name
```

**b. Configure the three networks on `wlan1` via netplan:**

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan1:
      dhcp4: true
      access-points:
        "ECE387":
          password: "ece387only"
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "20"
        "AF_ACADEMY_GUEST":
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "10"
        "ECE":
          password: "dfec3141"
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "5"
```

Higher `autoconnect-priority` wins when more than one network is in range: `ECE387` (lab) is preferred, then `AF_ACADEMY_GUEST` (internet), then `ECE` (backup internet).

**c. Netplan config files must not be world-readable** — they contain the WiFi passwords in plaintext. Ubuntu warns about this if permissions are too open:

```bash
sudo chmod 600 /etc/netplan/*.yaml
```

**d. Apply and verify:**

```bash
sudo netplan apply

ip a show wlan1
nmcli connection show --active
ping 10.99.1.50    # login server, over ECE387
```

**e. Reconnecting to `ECE387` manually, if `wlan1` is currently on a lower-priority network** (e.g. `ECE`) **and `ECE387` comes back into range:** NetworkManager doesn't auto-preempt an active connection, so switch it by hand:

```bash
nmcli connection up "ECE387"
```

> `wlo1` is intentionally left out of this config — it stays free for students to join their robot's AP each session via `nmcli dev wifi connect "robot_XX_ap" password "..." ifname wlo1`.

---

## 1a. Reconfiguring an Existing Master (New Server IP + WiFi Setup)

Use this on a master that was already set up under the old scheme (static Ethernet IP, server at `192.168.0.151`). It moves the master's networking to the current setup — built-in adapter free for the robot AP, USB dongle (renamed to `wlan1`) on `ECE387`/`AF_ACADEMY_GUEST`/`ECE` — and points authentication and home directories at the login server's new IP, `10.99.1.50`.

### Step 1: Rename the USB WiFi dongle to `wlan1`

Each master's dongle has a different MAC address, so match by bus/type instead so the same config works on every machine:

```bash
sudo nano /etc/systemd/network/10-usb-wifi.link
```

```ini
[Match]
Type=wlan
Property=ID_BUS=usb

[Link]
Name=wlan1
```

```bash
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=net --action=add
# Unplug and replug the dongle (or reboot) so it re-enumerates as wlan1
```

Confirm:

```bash
ip a   # should show wlan1 in place of the old wlx... name
```

### Step 2: Configure the three WiFi networks on `wlan1`

Ubuntu Desktop uses NetworkManager to manage networking, and netplan hands control to it. `ls /etc/netplan/` typically shows:

| File | What it is | Edit it? |
|------|------------|------------------|
| `01-network-manager-all.yaml` | Ships by default; tells netplan "let NetworkManager handle every interface." | No |
| `90-NM-*.yaml` (one or more) | Auto-generated by NetworkManager, one per connection profile. Rewritten automatically whenever a profile changes. | No — hand edits get overwritten |
| `50-cloud-init.yaml` | Written by the installer at provisioning. This is the one to edit. | **Yes** |

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

```yaml
network:
  version: 2
  renderer: NetworkManager
  wifis:
    wlan1:
      dhcp4: true
      access-points:
        "ECE387":
          password: "ece387only"
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "20"
        "AF_ACADEMY_GUEST":
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "10"
        "ECE":
          password: "dfec3141"
          networkmanager:
            passthrough:
              connection.autoconnect-priority: "5"
```

This replaces whatever was in the file before (old static Ethernet config, or an old home-WiFi profile) — all three networks use DHCP, with `ECE387` preferred, then `AF_ACADEMY_GUEST`, then `ECE` as backup.

Netplan files contain the WiFi passwords in plaintext, so fix permissions (Ubuntu will warn on `netplan apply` if this isn't done):

```bash
sudo chmod 600 /etc/netplan/*.yaml
```

Apply and confirm:

```bash
sudo netplan apply

ip a show wlan1
nmcli connection show --active
ping 10.99.1.1     # the router
ping 10.99.1.50    # the login server
```

Stop cloud-init from reverting this file on the next boot:

```bash
sudo nano /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```

```yaml
network: {config: disabled}
```

If `wlan1` later connects to a lower-priority network (e.g. `ECE`) and `ECE387` becomes available again, NetworkManager won't auto-switch — reconnect manually:

```bash
nmcli connection up "ECE387"
```

> `wlo1` (built-in) is intentionally left unconfigured — students point it at their robot's AP each session via `nmcli dev wifi connect "robot_XX_ap" password "..." ifname wlo1`.

### Step 3: Point SSSD at the new login server IP

```bash
sudo nano /etc/sssd/sssd.conf
```

Change the `ldap_uri` line to:

```ini
ldap_uri = ldap://10.99.1.50
```

```bash
# Restart SSSD and clear its cache so it doesn't hold onto the old server's stale data
sudo systemctl stop sssd
sudo rm -rf /var/lib/sss/db/*
sudo systemctl start sssd
sleep 5

# Test: should return the student's uid/gid, confirming the new LDAP URI works
id a27-m0
```

### Step 4: Point autofs at the new login server IP

```bash
sudo nano /etc/auto.students
```

Update the NFS source IP:

```
*  -fstype=nfs,soft,timeo=30,retrans=2  10.99.1.50:/home/students/&
```

```bash
sudo systemctl restart autofs

# Test: home directory should mount automatically
sudo su - a27-m0
pwd
exit
```

### Step 5: Confirm mDNS hostname resolution

Since the master's IP can change on every DHCP renewal, other machines (including the login server, for Ansible) reach it by hostname instead:

```bash
# Confirm avahi-daemon is running (ships by default on Ubuntu Desktop)
systemctl status avahi-daemon

# From another machine on the ECE387 network, confirm this master answers:
ping masterNN.local    # replace NN with this machine's number, e.g. master03.local
```

If you're reconfiguring all 14 masters, repeat Steps 1–2 on each (the `.link` file and `50-cloud-init.yaml` are identical across machines, so these can be pushed via Ansible too), and push Steps 3–4 via the Ansible playbook in [Section 4](#4-ansible-automation-recommended-for-14-machines). Do Steps 1–2 carefully if working over SSH — a network change can drop the connection mid-command if it affects the interface your SSH session is using.

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

AF_ACADEMY_GUEST WiFi provides internet access independently of the ECE lab network, so GitHub is reachable even when the login server is not.

### Option 2: Soft NFS Mounts

Without `soft`, a failed NFS mount causes the accessing process to hang indefinitely — the terminal freezes and the student cannot do anything. `soft` mounts fail fast instead, so the student gets an error message rather than a frozen session.

This is already included in the `/etc/auto.students` configuration in Section 1.4:

```
*  -fstype=nfs,soft,timeo=30,retrans=2  10.99.1.50:/home/students/&
```

If you need to change this after initial setup, edit the file and restart autofs:

```bash
sudo systemctl restart autofs
```

---

## 3. Student Workflow

When a student moves to a different station:

1. Log in with their LDAP username (e.g., `a27-m0`) and password — SSSD authenticates against the login server and their home directory mounts automatically via NFS.
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

# The login server is Ubuntu Server, which — unlike Ubuntu Desktop — does NOT
# ship with mDNS resolution by default. Install it so *.local hostnames resolve:
sudo apt install -y avahi-daemon libnss-mdns
sudo systemctl enable --now avahi-daemon

# Quick check: this should resolve to master01's current DHCP IP
ping -c 1 master01.local

# Create an inventory file listing all master computers.
# Masters now get dynamic IPs from the WiFi router, so we address them by
# their mDNS hostname (masterNN.local) instead of a static IP — this still
# resolves correctly even after a reboot changes the machine's DHCP lease.
cat > ~/Documents/ece387/masters-inventory.ini << 'EOF'
[masters]
master01 ansible_host=master01.local
master02 ansible_host=master02.local
master03 ansible_host=master03.local
master04 ansible_host=master04.local
master05 ansible_host=master05.local
master06 ansible_host=master06.local
master07 ansible_host=master07.local
master08 ansible_host=master08.local
master09 ansible_host=master09.local
master10 ansible_host=master10.local
master11 ansible_host=master11.local
master12 ansible_host=master12.local
master13 ansible_host=master13.local
master14 ansible_host=master14.local

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
# SSH into each master by its mDNS hostname and run 'who' to see logged-in users.
# masterNN.local resolves regardless of the master's current DHCP-assigned IP.
for i in $(seq -w 1 14); do
  echo "=== master$i ==="
  ssh ece387admin@master$i.local who 2>/dev/null
done
```

---

## Troubleshooting

| Problem | Command | Fix |
|---------|---------|-----|
| Home dir not mounting | `automount -v` | Check NFS exports on server, autofs config on master, server reachable |
| SSSD not resolving users | `sssctl user-checks a27-m0` | Restart sssd: `systemctl restart sssd` |
| NFS mount fails | `showmount -e 10.99.1.50` | Check firewall on server; verify `nfs-server` is running |
| Login hangs (no soft mount) | `journalctl -u autofs` | Ensure `soft,timeo=30,retrans=2` is in `/etc/auto.students` |
| Login slow (~30s) | `journalctl -u sssd` | Check LDAP URI is reachable: `ping 10.99.1.50` |
| Can't reach a master by hostname | `ping masterNN.local` | Check `avahi-daemon` is running on both machines: `systemctl status avahi-daemon` |

For server-side symptoms (student account issues, LDAP connection refused), see the troubleshooting table in [login-server.md](login-server.md).
