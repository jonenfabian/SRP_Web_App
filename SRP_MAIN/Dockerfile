# Base Image
#FROM python:3.6
FROM python:buster
# create and set working directory
RUN mkdir /app
WORKDIR /app

# Add current directory code to working directory
ADD . /app/

# set default environment variables
ENV PYTHONUNBUFFERED 1
ENV LANG C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive

# set project environment variables
# grab these via Python's os.environ
# these are 100% optional here
ENV PORT=8000

# Install system dependencies
#RUN apt-get update && apt-get install -y --no-install-recommends \
#        tzdata \
#        python3-setuptools \
#        python3-pip \
#        python3-dev \
#        python3-venv \
#        git \
#        && \
#    apt-get clean && \
#    rm -rf /var/lib/apt/lists/*


# install environment dependencies
#RUN pip3 install --upgrade pip
#RUN pip3 install pipenv
RUN pip3 install gunicorn
RUN pip3 install numpy
RUN pip3 install django
RUN pip3 install django-bootstrap4
RUN pip3 install matplotlib
#RUN pip3 install opencv-python
RUN pip3 install opencv-contrib-python
# Install project dependencies
#RUN pipenv install --skip-lock --system --dev

EXPOSE 8888
#CMD gunicorn SRP_MAIN.wsgi --bind 0.0.0.0:8000
CMD ["python", "manage.py", "runserver", "0.0.0.0:8000"]
