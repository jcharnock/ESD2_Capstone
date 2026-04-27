function sql_link(sqlStruct, image_source)
%SQL_LINK connects to the SQL database and sends shot data.
%
% This function receives a SQL-friendly MATLAB struct and an image path.
% It writes the shot data to MySQL.

arguments (Input)
    sqlStruct
    image_source
end

    % Set up entry number counter and database credentials.
    persistent entry_number;

    if isempty(entry_number)
        entry_number = 0;
    end

    username = "root";
    password = "tennistracker123";

    % Store the image path instead of the full image matrix.
    % This is safer for sqlwrite than trying to upload a 3D uint8 image.
    sqlStruct.court_image_path = string(image_source);

    % Convert struct to SQL table.
    shotTable = struct2table(sqlStruct);

    % Connect to SQL.
    try
        conn = mysql(username, password, ...
            'Server', "localhost", ...
            'DatabaseName', "volleys", ...
            'PortNumber', 3306);
    catch ME
        fprintf('SQL connection error: %s\n', ME.message);
        return;
    end

    % Write to database.
    entry_number = entry_number + 1;

    tableName = sprintf('serve_no_%d', entry_number);

    try
        sqlwrite(conn, tableName, shotTable);
        fprintf('SQL upload successful: %s\n', tableName);
    catch ME
        fprintf('SQL write error: %s\n', ME.message);
    end

    close(conn);
end