/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-web-server
 */

const char *HTML_CONTENT = R""""(

  <!DOCTYPE HTML>
  <html>
    <head>
      <link rel=\"icon\" href=\"data:,\">
      <style>
        * {
          font-size: 60px;
          text-align: center;
        }
      </style>
    </head>

    <body>
      <h1>
        Accelerometer:
      </h1>
      <p>
        X: %DATA0%<br>
        Y: %DATA1%<br>
        Z: %DATA2%
      </p>
      <h1>
        Magnetometer:
      </h1>
      <p>
        X: %DATA3%<br>
        Y: %DATA4%<br>
        Z: %DATA5%
      <p>

      <!-- Reloads page every 500 milliseconds -->
      <script>
        function reload() {
          window.location.reload(1);
        }
        setTimeout(reload, 500);
      </script>
    </body>
  </html>

)"""";
