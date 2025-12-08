const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>Line Bot Controller</title>
  <style>
    body {
      font-family: Arial;
      background:#222;
      color:white;
      text-align:center;
      padding-top:40px;
    }
    select, button {
      padding:10px;
      font-size:18px;
    }
  </style>
</head>
<body>
  <h1>Line Follower Controller</h1>

  <h3>Select Line Color</h3>
  <select id="colorSel">
    <option value="red">Red</option>
    <option value="green">Green</option>
    <option value="blue">Blue</option>
  </select>

  <button onclick="send()">Apply</button>

  <p>Accel X: %ACCEL_X%</p>
  <p>Accel Y: %ACCEL_Y%</p>
  <p>Accel Z: %ACCEL_Z%</p>


  <script>
    function send() {
      let c = document.getElementById("colorSel").value;
      fetch("/setColor?color=" + c)
        .then(r => r.text())
        .then(alert);
    }
  </script>
</body>
</html>
)rawliteral";
