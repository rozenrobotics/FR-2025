function renderBuilding(marker, id) {
    document.getElementById(id + "img").src = "/static/ico/building.png";
    document.getElementById(id + "color").style.backgroundColor = createjs.Graphics.getRGB(marker.color.r * 255, marker.color.g * 255, marker.color.b * 255);
    document.getElementById(id + "x").innerHTML = "x: " + (Math.round(marker.pose.position.x * 100) / 100).toString();
    document.getElementById(id + "y").innerHTML = "y: " + (Math.round(marker.pose.position.y * 100) / 100).toString();
    document.getElementById(id + "type").innerHTML = marker.text;
    document.getElementById(id + "number").innerHTML = "id = " + (marker.id + 1).toString();
}

function addBuildingLabel(id) {
    document.getElementById("buildings-list").innerHTML += "<div id='" + id + "' class='building-el'><div><img id='" + id
        + "img' class='building-img' alt='' src=''/>" +
        "</div><div class='elcontento'><div class='full'><strong>Position</strong>" +
        "<div><div id='" + id + "x'></div><div id='" + id + "y'></div><div class='number' id='" + id + "number'></div></div></div>" +
        "<div class='full'><strong>Type</strong>" +
        "<div><div id='" + id + "type'></div>" +
        "</div></div>" +
        "<div class='colorel' id='" + id + "color'>";
}

function removeAllBuildingLabels() {
    document.getElementById("buildings-list").innerHTML = '';
}