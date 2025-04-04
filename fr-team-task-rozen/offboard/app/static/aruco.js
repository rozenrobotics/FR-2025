// for ref. see https://chev.me/arucogen/

function generateMarkerSvg(width, height, bits, fixPdfArtifacts = true) {
    var svg = document.createElement('svg');
    svg.setAttribute('viewBox', '0 0 ' + (width + 2) + ' ' + (height + 2));
    svg.setAttribute('xmlns', 'http://www.w3.org/2000/svg');
    svg.setAttribute('shape-rendering', 'crispEdges');

    // Background rect
    var rect = document.createElement('rect');
    rect.setAttribute('x', 0);
    rect.setAttribute('y', 0);
    rect.setAttribute('width', width + 2);
    rect.setAttribute('height', height + 2);
    rect.setAttribute('fill', 'black');
    svg.appendChild(rect);

    // "Pixels"
    for (var i = 0; i < height; i++) {
        for (var j = 0; j < width; j++) {
            var white = bits[i * height + j];
            if (!white) continue;

            var pixel = document.createElement('rect');;
            pixel.setAttribute('width', 1);
            pixel.setAttribute('height', 1);
            pixel.setAttribute('x', j + 1);
            pixel.setAttribute('y', i + 1);
            pixel.setAttribute('fill', 'white');
            svg.appendChild(pixel);

            if (!fixPdfArtifacts) continue;

            if ((j < width - 1) && (bits[i * height + j + 1])) {
                pixel.setAttribute('width', 1.5);
            }

            if ((i < height - 1) && (bits[(i + 1) * height + j])) {
                var pixel2 = document.createElement('rect');;
                pixel2.setAttribute('width', 1);
                pixel2.setAttribute('height', 1.5);
                pixel2.setAttribute('x', j + 1);
                pixel2.setAttribute('y', i + 1);
                pixel2.setAttribute('fill', 'white');
                svg.appendChild(pixel2);
            }
        }
    }

    return svg;
}

function generateArucoMarker(id) {
    var bytes = arucoBytes[id];
    var bits = [];
    var bitsCount = 16;

    // Parse marker's bytes
    for (var byte of bytes) {
        var start = bitsCount - bits.length;
        for (var i = Math.min(7, start - 1); i >= 0; i--) {
            bits.push((byte >> i) & 1);
        }
    }

    var svg = generateMarkerSvg(4, 4, bits);
    svg.setAttribute('width', 5 + 'mm');
    svg.setAttribute('height', 5 + 'mm');
    return 'data:image/svg+xml;base64,' + btoa(svg.outerHTML.replace('viewbox', 'viewBox'));
}