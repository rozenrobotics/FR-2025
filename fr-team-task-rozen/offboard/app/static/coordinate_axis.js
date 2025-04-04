CoordinateAxis = function(options) {
    var width = options.width || 1;
    var height = options.height || 1;
    var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 100, 0);

    var graphics = new createjs.Graphics();
    graphics.setStrokeStyle(0.06);
    graphics.beginStroke(strokeColor);
    graphics.moveTo(0, 0);
    graphics.lineTo(0, height * 0.95);
    graphics.lineTo(-width/2, height * 0.95);
    graphics.lineTo(0, height);
    graphics.lineTo(width/2, height * 0.95);
    graphics.lineTo(0, height * 0.95);
  
    createjs.Shape.call(this, graphics);
};

CoordinateAxis.prototype.__proto__ = createjs.Shape.prototype;