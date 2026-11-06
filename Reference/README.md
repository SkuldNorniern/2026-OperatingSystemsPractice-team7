# Reference

Reference material used in the course. Nothing here is part of the project build.

| Path | Description |
|---|---|
| [`cspace_demo/`](cspace_demo/README.md) | Lecture demo code for configuration space (C-space): Minkowski sum, 3D C-space, 2-link arm |
| [`tb4_timesync_install.txt`](tb4_timesync_install.txt) | Setup script for syncing the clocks of the laptop, Raspberry Pi, and Create3 on TurtleBot4 |
| [`tb4_timesync/`](tb4_timesync/README.md) | chrony-based alternative: the laptop serves NTP to the RPi, and the RPi serves the Create3. One-time setup |

## Notes

- `tb4_timesync_install.txt` must keep Unix (LF) line endings. If it fails with a CRLF error, run `sed -i 's/\r$//' tb4_timesync_install.txt`.
- See each subfolder's README for install and run instructions.
