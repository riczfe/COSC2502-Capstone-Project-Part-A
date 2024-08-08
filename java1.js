const view = document.getElementById('stream');
        const WS_URL = "ws://" + window.location.host + ":82";
        const ws = new WebSocket(WS_URL);
        
        ws.onmessage = message => {
            if (message.data instanceof Blob) {
                var urlObject = URL.createObjectURL(message.data);
                view.src = urlObject;
            }
        };

        var lastText, lastSend, sendTimeout;
        function send(txt) {
            var now = new Date().getTime();
            if (lastSend === undefined || now - lastSend >= 30) {
                try {
                    ws.send(txt);
                    lastSend = new Date().getTime();
                    return;
                } catch (e) {
                    console.log(e);
                }
            }
            lastText = txt;
            if (!sendTimeout) {
                var ms = lastSend !== undefined ? 30 - (now - lastSend) : 30;
                if (ms < 0)
                    ms = 0;
                sendTimeout = setTimeout(() => {
                    sendTimeout = null;
                    send(lastText);
                }, ms);
            }
        }

        var joy = new JoyStick('joyDiv');
        var potSlider = document.getElementById("potSlider");

        function getMotorValues(nJoyX, nJoyY, sliderValue) {
            var nMotMixL;
            var nMotMixR;
            var fPivYLimit = 32.0;

            var nMotPremixL;
            var nMotPremixR;
            var nPivSpeed;
            var fPivScale;

            if (nJoyY >= 0) {
                nMotPremixL = (nJoyX >= 0 ? 100.0 : 100.0 + parseFloat(nJoyX));
                nMotPremixR = (nJoyX >= 0 ? 100.0 - nJoyX : 100.0);
            } else {
                nMotPremixL = (nJoyX >= 0 ? 100.0 - nJoyX : 100.0);
                nMotPremixR = (nJoyX >= 0 ? 100.0 : 100.0 + parseFloat(nJoyX));
            }

            nMotPremixL = nMotPremixL * nJoyY / 100.0;
            nMotPremixR = nMotPremixR * nJoyY / 100.0;

            nPivSpeed = nJoyX;
            fPivScale = (Math.abs(nJoyY) > fPivYLimit) ? 0.0 : (1.0 - Math.abs(nJoyY) / fPivYLimit);

            nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
            nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);

            return Math.round(nMotMixL * 2.55) + "," + Math.round(nMotMixR * 2.55) + "," + sliderValue;
        }

        potSlider.oninput = function() {
            var sliderValue = potSlider.value;
            send(getMotorValues(joy.GetX(), joy.GetY(), sliderValue));
        }

        function sendEmergencyStop() {
            try {
                ws.send("EMERGENCY_STOP");
            } catch (e) {
                console.log(e);
            }
        }

        setInterval(function () {
            send(getMotorValues(joy.GetX(), joy.GetY(), potSlider.value));
        }, 300);