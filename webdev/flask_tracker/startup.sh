#!/bin/bash

trap "kill 0" EXIT

cd backend
python3 app.py &
cd ../frontend
python3 app.py &

wait