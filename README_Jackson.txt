All webdev and SQL interfacing done by Jackson (see /webdev), along with 
documentation in the User's Manual and CDR Technical documentation. Contributed to 
Powerpoint slides as well. Minor contributions to the GUI, also responsible for the 
board setup and .sh scripts in /webdev.

In /webdev:
The flask_tracker folder is the active working folder for the User GUI 
python application. Documented and described in the CDR documentation.

The SQLTest folders contain initial .m scripts for testing connectivity from
MATLAB to the MySQL database which required figuring out how a SQL works 
along with how to connect to databases in MATLAB and Python. MATLAB required the 
SQL Database Toolbox add on.

The web_tennis_tracker folder contains initial experimentation with Django for
backend webserver via Python and a earlier (non-working) iteration of the frontend
app.py found in the flask_tracker project. Django backend development was abandoned
due to Django's complexity requiring more time due to how robust the framework needs
to be, and Flask was adopted instead due to the simplicity of the Flask framework
that more aligned with the simplicity of the User GUI app.