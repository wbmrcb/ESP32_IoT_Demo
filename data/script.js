
var accelChart = document.getElementById('accelChart').getContext('2d');
var magnitudeChart = document.getElementById('magnitudeChart').getContext('2d');
var minMagnitude = document.getElementById('minMagnitude');
var clearAlerts = document.getElementById('clearAlerts');
var freeFallAlert = document.getElementById('freeFallAlert');

var ip = window.location.hostname;
if (ip === "127.0.0.1" || ip === "localhost") ip = "192.168.8.108";
console.log("Using: " + ip);

var socket = new WebSocket(`ws://${ip}/ws`);

var accChart = new Chart(accelChart, {
    type: 'line',
    data: {
        datasets: [
            { label: 'x', data: [], borderColor: "#FF0063", lineTension: 0, pointRadius: 0 },
            { label: 'y', data: [], borderColor: "#63FF00", lineTension: 0, pointRadius: 0 },
            { label: 'z', data: [], borderColor: "#0063FF", lineTension: 0, pointRadius: 0 },
        ]
    }
});

var magChart = new Chart(magnitudeChart, {
    type: 'line',
    data: {
        datasets: [
            { label: 'magnitude', data: [], borderColor: "#FF0063", lineTension: 0, pointRadius: 0 },
            { label: 'mDiff', data: [], borderColor: "#63FF00", lineTension: 0, pointRadius: 0 },
        ]
    }
});

var lastMagnitude = 0;
var dataPoint = 0;
var lastFreefall = 0;

socket.onmessage = (event) => {
    accChart.data.labels.push(dataPoint);
    magChart.data.labels.push(dataPoint);
    dataPoint++;

    var acc = event.data.split(' ');
    var magnitude = acc.reduce((sum, i) => Math.abs(sum) + Math.abs(i));
    var gdiff = magnitude - lastMagnitude;
    lastMagnitude = magnitude;

    magChart.data.datasets[0].data.push(magnitude);
    magChart.data.datasets[1].data.push(gdiff);

    accChart.data.datasets.forEach((dataset, i) => {
        accChart.data.datasets[i].data.push(acc[i]);
    })

    if (accChart.data.datasets[0].data.length >= 100) {
        accChart.data.datasets.forEach((dataset, i) => {
            accChart.data.datasets[i].data.shift();
        })

        magChart.data.datasets[0].data.shift();
        magChart.data.datasets[1].data.shift();

        accChart.data.labels.shift();
        magChart.data.labels.shift();
    }

    accChart.update();
    magChart.update();

    var minMag = Math.min(...magChart.data.datasets[0].data);
    minMagnitude.innerHTML = "Min Magnitude: " + minMag.toFixed(2);

    if (magnitude < 1.5 && (dataPoint - lastFreefall) > 30) {
        lastFreefall = dataPoint;
        freeFallAlert.innerHTML = freeFallAlert.innerHTML + "Freefall at:" + dataPoint + "</br>";
    }
}

clearAlerts.onclick = () => {
    freeFallAlert.innerHTML = "";
}
