from flask import Flask, Response, render_template, url_for
from api import api_bp
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

@app.route('/Globe_Run')
def globe_run():
    return render_template('globe_run.html')

if __name__ == '__main__':
    print("Starting Flask server with camera monitoring...")
    print("Camera interface will be available at: http://localhost:5000/cameras")
    print("Demo mode available with Ctrl+D if no cameras are connected")
    app.run(debug=True, host='0.0.0.0', port=5000)