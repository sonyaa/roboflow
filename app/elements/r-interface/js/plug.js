function Plug(c, n, name, color, radius, offsetX, offsetY) {
    var centerX = n.left + n.width / 2;
    var centerY = n.top + n.height / 2;
    this.radius = radius;

    var thiz = this;

    var callbacks = [];

    this.canvas = c;
    this.node = n;
    this.isPlug = true;
    this.name = name;
    this.edge = null;

    this.container = new fabric.Circle({
        radius: this.radius,
        left: centerX + offsetX,
        top: centerY + offsetY,
        originX: 'center',
        originY: 'center',
        fill: color,
        selectable: false,
        hasControls: false,
        hasBorders: false
    });
    thiz.canvas.add(thiz.container);
    thiz.container.bringToFront();

    thiz.container.on('mousedown', function (e) {
        //Ensure clicking plugs doesn't de-select the object they're attached to
        thiz.canvas.setActiveObject(thiz.node);
        if (thiz.edge !== null) {
            thiz.edge.dispose();
        }
        var edge = new Edge(thiz.canvas, thiz);
        edge.startConnection();
    });

    this.updateNode = function(n) {
        thiz.node = n;
        thiz.refresh(n.left + n.width / 2, n.top + n.height / 2);
    };

    this.refresh = function() {
        var centerX = thiz.node.left + thiz.node.width / 2;
        var centerY = thiz.node.top + thiz.node.height / 2;

        thiz.container.left = centerX + offsetX;
        thiz.container.top = centerY + offsetY;
        thiz.container.setCoords();
    };

    thiz.node.on('moving', function(e) {
        thiz.refresh();
    });


    this.addListener = function (callback) {
        thiz.container.on('mousedown', function (e) {
            //Ensure clicking plugs doesn't de-select the object they're attached to
            thiz.canvas.setActiveObject(thiz.node);

            callback(e);
        });

        callbacks.push(callback);
    };

    this.dispose = function() {
        thiz.container.off();
        thiz.container.remove();
        if (thiz.edge != null) {
            thiz.edge.dispose();
        }

        callbacks = [];
    };

}