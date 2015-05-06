function getSocket(c, n, name, color, radius, offsetX, offsetY) {
    var centerX = n.left + n.width / 2;
    var centerY = n.top + n.height / 2;
    var thickness = 2;

    var socket = new fabric.Circle({
        radius: radius,
        left: centerX + offsetX,
        top: centerY + offsetY,
        originX: 'center',
        originY: 'center',
        stroke: color,
        strokeWidth: thickness,
        fill: 'white',
        selectable: false,
        hasControls: false,
        hasBorders: false
    });

    socket.selector = new fabric.Circle({
        radius: radius + 3,
        left: -100,
        top: -100,
        originX: 'center',
        originY: 'center',
        stroke: 'yellow',
        strokeWidth: thickness,
        fill: "transparent",
        selectable: false,
        hasControls: false,
        hasBorders: false
    });

    socket.canvas = c;
    socket.node = n;
    socket.isSocket = true;
    socket.name = name;

    socket.show = function() {
        socket.canvas.add(socket);
    };

    socket.show();

    socket.refresh = function() {
        var centerX = socket.node.left + socket.node.width / 2;
        var centerY = socket.node.top + socket.node.height / 2;

        socket.left = centerX + offsetX;
        socket.top = centerY + offsetY;
    };

    socket.node.on('moving', function(e) {
        socket.refresh();
    });


    socket.dispose = function() {
        if (socket.edge != null) {
            socket.edge.dispose();
        }
        socket.off();
        socket.remove();
    };
    //socket.canvas.sockets[]

    return socket;
}