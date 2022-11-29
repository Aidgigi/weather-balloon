FROM 3.10-bullseye
WORKDIR /wdir
COPY requirements.txt requirements.txt
RUN pip install -r requirements.
EXPOSE 8009
COPY . .
CMD ["python", "server.py"]