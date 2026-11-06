# tb4_timesync

Keeps the host laptop, the TurtleBot4 Raspberry Pi, and the Create3 on the same clock using chrony (NTP). Target: Ubuntu 24.04 with ROS 2 Jazzy on both machines.

```
host laptop ──NTP──▶ RPi ──NTP (USB-C, 192.168.186.0/24)──▶ Create3
```

If the clocks drift apart, TF lookups fail with "extrapolation into the future/past" errors, and SLAM or Nav2 drops scans.

## How this differs from `../tb4_timesync_install.txt`

| | Course script | These scripts |
|---|---|---|
| Mechanism | `date -s` over SSH, run by hand | chrony runs in the background on every machine |
| When to run | Every time the robot boots | Once (setup) |
| Laptop clock | Moved to match the Create3, with NTP turned off | Unchanged, keeps internet NTP |
| Needs on the robot | SSH key and passwordless `sudo date` | Nothing extra |

Use one approach or the other. Running the course script turns off chrony on the laptop, which stops it serving time.

## Files

| File | Runs on | Purpose |
|---|---|---|
| `tb4_host_timeserver.sh` | Laptop | Makes the laptop an NTP server for the robot subnet |
| `tb4_rpi_timesync.sh` | RPi | Makes the RPi follow the laptop and keep serving time to the Create3 |
| `tb4_timecheck.sh` | Laptop | Measures `/odom` and `/scan` stamp offsets (read-only) |

## Setup (once)

1. On the laptop (default robot subnet is `192.168.185.0/24`):

   ```bash
   chmod +x tb4_*.sh
   sudo ./tb4_host_timeserver.sh setup            # or: setup 10.0.0.0/24
   ```

   Note the laptop IP it prints on the robot network.

2. Copy the RPi script to the robot and run it with that IP:

   ```bash
   scp tb4_rpi_timesync.sh ubuntu@192.168.185.3:~
   ssh ubuntu@192.168.185.3
   sudo ./tb4_rpi_timesync.sh setup <LAPTOP_IP>
   ```

3. Check that the Create3 uses the RPi as its time server. In the Create3 web UI (`http://192.168.186.2`, reachable from the RPi), its NTP configuration should contain `server 192.168.186.3 iburst`. This is the TurtleBot4 default. If you change it, restart the Create3 application.

## Daily use

Nothing to run. The RPi steps its clock as soon as it reaches the laptop after boot. To confirm:

```bash
./tb4_timecheck.sh                       # laptop: /odom and /scan should be ~0.0xx s
sudo ./tb4_rpi_timesync.sh status        # RPi: offset, sources, Create3 as client
sudo ./tb4_rpi_timesync.sh sync          # RPi: force an immediate resync
```

A clock step while SLAM, Nav2 or RViz is running confuses them, so restart them after a large correction.

## Troubleshooting

- **RPi sync times out.** Check that the laptop IP in `/etc/chrony/conf.d/50-tb4-host.conf` is correct, UDP port 123 is open on the laptop (the setup script opens it if `ufw` is active), and `sudo ./tb4_host_timeserver.sh status` lists the RPi as a client.
- **`/scan` is OK but `/odom` is off.** The Create3 isn't following the RPi. Check its NTP config (step 3) and restart the Create3 application.
- **Undo.** Run `sudo ./tb4_rpi_timesync.sh restore` on the RPi and `sudo ./tb4_host_timeserver.sh restore` on the laptop.
- **CRLF errors** (`$'\r': command not found`): run `sed -i 's/\r$//' tb4_*.sh`.
