from flask import Flask
from flask import render_template

app = Flask(__name__)
# TODO: https://flask.palletsprojects.com/en/3.0.x/quickstart/#http-methods

# To Run server:
# python -m flask --app frcdash run


@app.route("/")
def hello_world():
    return render_template('hello.html')
