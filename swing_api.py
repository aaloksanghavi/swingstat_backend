# import main Flask class and request object
from flask import Flask, request
import AnalyzeSwing as analyze

# create the Flask app
app = Flask(__name__)

@app.route('/swing', methods=['POST'])
def json_example():
    request_data = request.get_json()

    output = analyze.analyze_swing(request_data)

    return output
if __name__ == '__main__':
    # run app in debug mode on port 5000
    app.run(debug=True, port=5000)
