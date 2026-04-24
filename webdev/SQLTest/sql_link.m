function sql_link(sqlStruct, image_source)
%   SQL_LINK connects to the SQL database to send shot data
%   Using the MySQL function to connect to outstanding MySQL database.
%   Pushes the entry number, timestamp, x & y coordinates, ruling, and
%   court image
arguments (Input)
    sqlStruct
    image_source
end

    % set up entry number counter and the username and password
    % DO NOT CHANGE USERNAME AND PASSWORD
    persistent entry_number;
    username = "root";
    password = "tennistracker123";

    % add image to the SQL struct based on image path
    imgData = imread(image_source);
    sqlStruct.court_image = imgData;

    % convert struct to SQL table
    table = struct2table(sqlStruct);

    % connect to SQL, print error if failed.
    try
        conn = mysql(username, password, 'Server', "localhost", ...
        'DatabaseName', "volleys", 'PortNumber', 3306);
    catch ME
        fprintf('An error occurred: %s\n', ME.message);
        return
    end

    % write out to database if connection works & close connection
    entry_number = entry_number + 1;
    entryno = "serve_no_" + entry_number;
    sqlwrite(conn, entryno, table);
    close(conn);
    
end