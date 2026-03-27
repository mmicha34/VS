# drone_project

Cleaned-up versions of the main MAVLink scripts from your ChatGPT conversation.

Suggested run order:

1. `python .\drone_project\check_sysids.py`
2. `python .\drone_project\check_msg_sysids.py`
3. `python .\drone_project\dual_status.py`
4. `python .\drone_project\test_takeoff_drone1.py`
5. `python .\drone_project\test_takeoff_drone2.py`
6. `python .\drone_project\drone1_to_drone2_follow.py`

Notes:

- These scripts assume ArduCopter-style `GUIDED` mode and local MAVLink outputs on UDP ports `14550` and `14560`.
- The helper module `mav_helpers.py` adds heartbeat checks, target setup, mode validation, arm waiting, and altitude monitoring.
- The older TCP-only debug scripts from the conversation were not copied because they were mostly temporary troubleshooting steps.
