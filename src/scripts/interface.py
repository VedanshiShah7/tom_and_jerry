from flask import Flask, request, redirect, url_for, render_template_string, session
import random

app = Flask(__name__)
app.secret_key = 'your_unique_secret_key'  # Ensure you use a unique secret key
app.config['SESSION_COOKIE_SECURE'] = False  # Disable secure cookies for local development

# Home route
@app.route("/")
def home():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Tom and Jerry Game</title>
    </head>
    <body>
        <h1>Welcome to the Tom and Jerry Game!</h1>
        <p>Are you ready to play?</p>
        <a href="{{ url_for('select_color') }}">Start Game</a>
    </body>
    </html>
    """)

# Cheese color selection route
@app.route("/select_color", methods=["GET", "POST"])
def select_color():
    if request.method == "POST":
        cheese_color = request.form["cheese_color"]
        session["cheese_color"] = cheese_color  # Store the cheese color in session
        return redirect(url_for("loading"))
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Select Cheese Color</title>
    </head>
    <body>
        <h1>Select the Cheese Color</h1>
        <form method="post">
            <label>
                <input type="radio" name="cheese_color" value="blue" required> Blue Cheese
            </label><br>
            <label>
                <input type="radio" name="cheese_color" value="yellow" required> Yellow Cheese
            </label><br>
            <button type="submit">Confirm</button>
        </form>
    </body>
    </html>
    """)

# Loading screen route
@app.route("/loading")
def loading():
    # Simulate loading screen and determine results
    tom_score = random.randint(0, 30)
    jerry_score = random.randint(0, 30)

    # Store the scores in session
    session["tom_score"] = tom_score
    session["jerry_score"] = jerry_score

    if tom_score > jerry_score:
        session["winner"] = "Tom"
        session["tie"] = False
    elif jerry_score > tom_score:
        session["winner"] = "Jerry"
        session["tie"] = False
    else:
        session["tie"] = True
        session["winner"] = None

    return redirect(url_for("results"))

# Results route
@app.route("/results")
def results():
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Game Results</title>
    </head>
    <body>
        {% if session['tie'] %}
            <h1>It's a Tie!</h1>
            <p>Both Tom and Jerry scored {{ session['tom_score'] }} points.</p>
        {% else %}
            <h1>{{ session['winner'] }} Wins!</h1>
            <p>Scores:</p>
            <ul>
                <li>Tom: {{ session['tom_score'] }}</li>
                <li>Jerry: {{ session['jerry_score'] }}</li>
            </ul>
        {% endif %}
        <a href="{{ url_for('reset') }}">Play Again</a>
    </body>
    </html>
    """)

# Reset route
@app.route("/reset")
def reset():
    # Reset the session game state
    session.clear()  # Clears all session data
    return redirect(url_for("home"))

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
