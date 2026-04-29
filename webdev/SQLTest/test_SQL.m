% user and password DO NOT CHANGE
username = "root";
password = "tennistracker123";

% dummy struct setup
sqlShot.entry_number = 0;
sqlShot.timestamp = 0;
sqlShot.x_coordinate = 0;
sqlShot.y_coordinate = 0;
sqlShot.ruling = "null";

for index = 1:10
    %convert data to table
    %connect to mySQL database
    conn = mysql(username,password,'Server',"localhost", ...
    'DatabaseName',"volleys",'PortNumber',3306)
    %create dummy data
    entryno = "serve_no_" + index;
    sqlShot.entry_number = index;
    sqlShot.timestamp = datetime('now');
    sqlShot.x_coordinate = rand() * 100; % Example random coordinate
    sqlShot.y_coordinate = rand() * 100; % Example random coordinate
    sqlShot.ruling = "True"; % Example ruling
    table = struct2table(sqlShot);
    %pass table to SQL
    sqlwrite(conn, entryno, table);
end

close(conn);