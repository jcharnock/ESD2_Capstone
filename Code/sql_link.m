function sql_link(sqlStruct, image_source)
%SQL_LINK connects to the SQL database and sends shot data.

arguments (Input)
    sqlStruct
    image_source
end

    persistent entry_number;

    if isempty(entry_number)
        entry_number = 0;
    end

    username = "root";
    password = "tennistracker123";

    % Do NOT read the image into SQL.
    % sqlStruct.court_image already contains the relative image path.
    shotTable = struct2table(sqlStruct);

    try
        conn = mysql(username, password, ...
            'Server', "localhost", ...
            'DatabaseName', "volleys", ...
            'PortNumber', 3306);
    catch ME
        fprintf('SQL connection error: %s\n', ME.message);
        return;
    end

    entry_number = entry_number + 1;
    tableName = "serve_no_" + entry_number;

    try
        sqlwrite(conn, tableName, shotTable);
        fprintf('SQL upload successful: %s\n', tableName);
    catch ME
        fprintf('SQL write error: %s\n', ME.message);
    end

    close(conn);
end