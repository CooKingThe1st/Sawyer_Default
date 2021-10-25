from flask import request
@app.route("/test", methods=["POST"])
def test():
    name_of_slider = request.form["name_of_slider"]
    return name_of_slider
