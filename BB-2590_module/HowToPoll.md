BB-2590 Polling Using RasperryPi3 (RPi3) and MySQL

1) Turn the RPi3 on and ensure the male connector is plugged into the female connector on the BB-2590.
2) Test the internet connectivity of the RPi3. After performing <ifconfig>, the IP address of the RPi3 should be on EECSNET (192.168.200.XXX)
3) Open a terminal and navigate to the following directory: <datagathering/ay17_capstones_GVR_Bot/SQL_ParserCode>. Execute <pwd> to print your current working directory.
4) The SQL_Parser code is version controlled. Depending on the version, execute <python version.py> where "version.py" is the most up-to-date version of the code.
5) After running the code, it will prompt the user to enter various values into a table. This includes the name of the table, the battery number, the type of test, and the speed of the test. Complete the table to start polling the BB-2590. 
--> If you enter the same table name as a table that already exists, the code will over-write the old version of the table.
--> Please reference "BB-2590 Register Information" PDF in the "Datasheet" folder on the "EE489 Battery State Prediction" Google Drive for a comprehensive list of registers that the RPi3 can poll on the BB-2590.
6) There are two ways to access the MySQL database where the polling information is being stored:
METHOD A: Command Line
a) <mysql -u root -p> starts mysql with username "root" and will prompt you for a password
b) <show databases;> displays all the names of the databases
c) <use database;> enters within the database with name "database"
d) <show tables;> displays the name of all of the tables within the database
e) <select THIS from TABLE where CONDITIONS;> where "THIS" corresponds to the name of a column (ex: "remcap"), "TABLE" corresponds to the name of the table, and "CONDITIONS" correspond to any conditions you would like to add. You can omit the conditions statement at the end: <select THIS from TABLE> but it will select all values in this column.
METHOD B: User Interface
a) Open internet browser and type in: "localhost/phpmyadmin"
b) This should take you to a login page. Enter the username and password.
c) Navigate to the respective database and table.
d) To export data from the table, click on the table you would like to export information from and select "export" at the top. Files are best exported as .CSV and then converted into a more readable format later.
