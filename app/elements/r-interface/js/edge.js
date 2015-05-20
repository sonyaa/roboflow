function Edge(c, p) {
    var thiz = this;
    var canvas = c;
    var group = null;

    thiz.adorners = new Adorners(canvas, p.node, false);
    thiz.adorners.addAdorner('close', 'red', 1, 1, Icon.CLOSE, 0);
    thiz.adorners.addListener('close', function(e) {
        if(e.e.which === MouseButton.LEFT) {
            thiz.dispose();
        }
    });

    //Edges track the "from" plug and "to" socket
    this.from = p;

    var padding = 0;//10;

    var fromX = thiz.from.container.left + 2 * thiz.from.radius;
    var fromY = thiz.from.container.top + thiz.from.radius;
    var toX = thiz.from.container.left + 2 * thiz.from.radius;
    var toY = thiz.from.container.top + thiz.from.radius;

    this.drawArrow = function(x1, y1, x2, y2, dashed, width) {
        width = typeof width !== 'undefined' ? width : 6;
        var select = typeof this.to !== 'undefined';

        dashed = typeof dashed !== 'undefined' ? dashed : false;
        if(dashed) {
            width /= 2;
        }

        canvas.remove(group);

        var endOffset = x2 < x1 ? 25 : -20;

        var connector = paths.Connector({
            start: [x1, y1],
            end: [x2 + endOffset, y2]
        });

        // If edge connects a node to itself, make it go around the node
        if (this.to && this.to.node == this.from.node) {
            if (Math.abs(x2-x1) <= 10) {
                x2 = x1;
            }
            var path = paths.Path()
                .moveto(x1, y1)
                .lineto(x1 + this.to.node.width/2, y1)
                .lineto(x1 + this.to.node.width/2, y2)
                .lineto(x2 + endOffset, y2);
            connector = Object();
            connector.path = path;
        }

        var arrow = null;

        if(dashed) {
            arrow = new fabric.Path(connector.path.print(), {
                fill: '',
                stroke: 'green',
                strokeWidth: width,
                strokeDashArray: [10,10],
                selectable: select
            });
        } else {
            arrow = new fabric.Path(connector.path.print(), {
                fill: '',
                stroke: 'green',
                strokeWidth: width,
                selectable: select
            });
        }

        var polygon = paths.Polygon({
            points: [[x2 + endOffset, y2 - 10 + (width / 2)], [x2 + endOffset, y2 + 10 + (width / 2)], [x2, y2 + (width / 2)]],
            closed: true
        });

        var point = new fabric.Path(polygon.path.print(), {
            fill: 'green',
            selectable: select
        });

        group = new fabric.Group([arrow, point], {
            ElementType: ElementType.EDGE,
            hasControls: false,
            hasBorders: false,
            lockMovementX: true,
            lockMovementY: true
        });

        thiz.adorners.updateTarget(group);

        //group.on('selected', function(e) {
        //    console.log('Arrow selected');
        //});

        canvas.add(group);
        canvas.sendToBack(group);
    };

    this.update = function() {
        setFromTo();

        thiz.drawArrow(fromX, fromY, toX, toY);
    };

    function handleError(msg, isHTML) {
        console.log(msg);
        document.querySelector('r-interface').error(msg, isHTML);
    }

    function checkConstraints() {
        return true;
    }

    this.connect = function(target) {
        canvas.off('mouse:move', thiz.dragLine);
        canvas.forEachObject(function(object) {
            if(object.isSocket) {
                canvas.remove(object.selector);
            }
        });

        canvas.off('mouse:down', generateConnector);
        canvas.remove(group);

        if(target && target.isSocket) {
            thiz.to = target;

            if(!checkConstraints()) {
                //There was a constraint violation - return without creating the edge!
                return;
            }

            thiz.from.node.on('moving', thiz.update);
            thiz.to.node.on('moving', thiz.update);

            thiz.from.node.edges.outgoing.push(thiz);
            thiz.to.node.edges.incoming.push(thiz);

            thiz.from.node.updateState();
            thiz.to.node.updateState();

            thiz.from.edge = thiz;
            thiz.to.edge = thiz;

            thiz.update();

            canvas.renderAll();
        }
    };

    var generateConnector = function(p) {
        thiz.connect(p.target);
    };

    var setFromTo = function(pointerX, pointerY) {
        var toTargetX = null;

        if(typeof pointerX !== 'undefined') {
            toTargetX = pointerX;
        } else {
            toTargetX = thiz.to.left;

            if(toTargetX + thiz.to.radius < thiz.from.container.left) {
                toTargetX += thiz.to.radius;
            } else {
                toTargetX -= thiz.to.radius;
            }
        }

        var toTargetY = null;

        if(typeof pointerY !== 'undefined') {
            toTargetY = pointerY;
        } else {
            toTargetY = thiz.to.top;
        }

        fromX = thiz.from.container.left;
        fromY = thiz.from.container.top;
        toX = toTargetX;
        toY = toTargetY;
    };

    this.dragLine = function (p) {
        if(p.target && p.target.isSocket) {
            setFromTo(p.target.left, p.e.offsetY);
            thiz.drawArrow(fromX, fromY, toX, toY, true);
        } else {
            setFromTo(p.e.offsetX, p.e.offsetY);
            thiz.drawArrow(fromX, fromY, toX, toY, true);
        }
    };

    this.startConnection = function() {
        canvas.on('mouse:move', thiz.dragLine);
        canvas.on('mouse:down', generateConnector);
        canvas.forEachObject(function(object) {
            if(object.isSocket) {
                object.selector.top = object.top;
                object.selector.left = object.left;
                canvas.add(object.selector);
                object.selector.sendToBack();
                object.bringToFront();
            }
        });


        fromX = thiz.from.left + thiz.from.width;
        fromY = thiz.from.top + thiz.from.height / 2;
        toX = thiz.from.left + thiz.from.width;
        toY = thiz.from.top + thiz.from.height / 2;
    };

    this.dispose = function() {
        canvas.off('mouse:move', thiz.dragLine);
        canvas.off('mouse:down', generateConnector);

        canvas.remove(group);

        thiz.from.edge = null;
        thiz.from.node.off('moving', thiz.update);
        thiz.from.node.edges.outgoing.splice($.inArray(thiz, thiz.from), 1);

        if(thiz.to) {
            thiz.to.edge = null;
            thiz.to.node.off('moving', thiz.update);
            thiz.to.node.edges.incoming.splice($.inArray(thiz, thiz.to), 1);
        }

        thiz.adorners.dispose();
    };
}
