mapboxgl.accessToken = 'pk.eyJ1IjoiZ29kZWdrb2xhIiwiYSI6ImNsb2EwaWVzcTBmdHAycXFicTlsMmxyeXYifQ.wEYJUoOoqnFzHFURicvCgQ';

var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v11',
    zoom: 4,
    center: [120.984222, 14.599512]
});

// Initialize the marker and hide it on first load
var marker = new mapboxgl.Marker()
    .setLngLat([120.984222, 14.599512])
    .addTo(map);

// Hide the marker on first load
var markerElement = marker.getElement();
markerElement.style.display = 'none';

// Function to update the map center and marker position
function showMap(longitude, latitude) {
    // Move the map center to the specified location
    map.flyTo({
        center: [longitude, latitude],
        zoom: 14,
        essential: true // Ensures smooth transition
    });

    // Update the marker position to the new coordinates
    marker.setLngLat([longitude, latitude]);

    // Optionally, show the marker (for example, after updating the map center)
    markerElement.style.display = 'block'; // Show the marker when needed
}
