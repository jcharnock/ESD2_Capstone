## Installs the requirements needed for the web app

## Install packages
pip install -r docs/requirements.txt

## Run backend
cd backend
python manage.py migrate
python manage.py runserver

## Add sample data in Django shell
python manage.py shell

from backend.models import TennisEntry
TennisEntry.objects.create(
    entry_number=1,
    timestamp='00:00',
    x_coordinate=-10,
    y_coordinate=10,
    ruling='IN',
    court_image='../frontend/assets/placeholder_court.png'
)
