var mymap = L.map('mapid').setView([47.7511, -120.7401], 5);
L.tileLayer('https://api.tiles.mapbox.com/v4/{id}/{z}/{x}/{y}.png?access_token={accessToken}', {
    attribution: 'Map data &copy; <a href="https://www.openstreetmap.org/">OpenStreetMap</a> contributors, <a href="https://creativecommons.org/licenses/by-sa/2.0/">CC-BY-SA</a>, Imagery © <a href="https://www.mapbox.com/">Mapbox</a>',
    maxZoom: 20,
    id: 'mapbox.streets',
    accessToken: 'pk.eyJ1IjoiYWlkZ2lnaSIsImEiOiJjbGF0aXBoMnIwMTRmM25xZ3Bkd2twaTZ5In0.38nka8epFR_jw2be2YEuCQ' //ENTER YOUR ACCESS TOKEN HERE
}).addTo(mymap);

var source = new EventSource('/topic/coords'); //ENTER YOUR TOPICNAME HERE
source.addEventListener('message', function(e){

  console.log('Message');
  obj = JSON.parse(e.data);
  console.log(obj);

  L.marker([obj.latitude, obj.longitude]).addTo(mymap)
}, false);