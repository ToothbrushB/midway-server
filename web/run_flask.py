from flask import Flask, Response, render_template, url_for
from web.api import api_bp
app = Flask(__name__)
@app.route('/design')
def design():
    return render_template('design.html')

@app.route('/cameras')
def cameras():
    return render_template('cameras.html')

@app.route('/settings')
def settings():
    return render_template('settings.html')

@app.route('/tools')
def tools():
    return render_template('tools.html')

@app.route('/')
def index():
    return render_template('index.html')

app.add_url_rule(
    "/favicon.ico",
    endpoint="favicon",
    redirect_to="/static/favicon.ico"
)
app.register_blueprint(api_bp, url_prefix='/api')