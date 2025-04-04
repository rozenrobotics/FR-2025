Marker = function(options) {
    var width = options.width || 1;
    var height = options.height || 1;
    var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 100, 0);

    var graphics = new createjs.Graphics();
    graphics.beginStroke(strokeColor).drawRect(0, 0, width, height);
  
    createjs.Shape.call(this, graphics);
};

Marker.prototype.__proto__ = createjs.Shape.prototype;