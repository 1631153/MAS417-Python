# MAS417-Python
Project 2026, Flexible Production, Software-defined 3D-printable pendulum kit


Customer orders a pendulum kit through the GUI. The GUI is opened by running app.py, and ctrl+Left_click on the link provided in the terminal. 



Start spoolman (need Docker Desktop downloaded and running)

cd Flask/spoolman
docker compose up -d

Spoolman runs on http://localhost:7912 .


Output
For each order the program Writes:
	3MF files for all parts ( SIM_models/sim_model_<id>/ )
	the simulation video (Flask/static/Pendulum_<id>.mp4)
the swing period error and filament use per spool (Terminal)