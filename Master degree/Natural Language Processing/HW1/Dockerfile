FROM python:3.7-stretch

WORKDIR /home/app

# install requirements

COPY requirements.txt .
RUN pip install -r requirements.txt

# copy model

COPY model model

# copy code

COPY hw1 hw1
ENV PYTHONPATH hw1

# standard cmd

CMD [ "python", "hw1/app.py" ]
