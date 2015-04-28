// Author: Alexander Fiannaca

function Adorner(c, t, name, fill, ord, total, icon, rad) {

    if (typeof rad === 'undefined') {
        // It has not... perform the initialization
        rad = Math.sqrt(t.width * t.width + t.height * t.height) / 2 + 12;
    }

    var thiz = this;

    var callbacks = [];

    this.canvas = c;
    this.target = t;

    var centerX = t.left + t.width / 2;
    var centerY = t.top + t.height / 2;

    this.angle = selectPlacementAngle(ord, total);
    this.animateAngle = this.angle + Math.PI / 2;

    this.radius = rad;
    var thickness = 2;

    this.isClearing = false;

    this.container = new fabric.Circle({
        radius: 5,
        left: centerX + Math.cos(thiz.angle) * (thiz.radius * 1.1),
        top: centerY + Math.sin(thiz.angle) * (thiz.radius * 1.1),
        originX: 'center',
        originY: 'center',
        fill: fill,
        selectable: false,
        hasControls: false,
        hasBorders: false
    });

    this.innercontainer = new fabric.Circle({
        radius: thiz.container.radius - thickness,
        left: thiz.container.left,
        top: thiz.container.top,
        originX: 'center',
        originY: 'center',
        fill: 'white',
        selectable: false,
        hasControls: false,
        hasBorders: false
    });

    var closeSvg = '<svg height="20" width="20"><g><path d="M19 6.41l-1.41-1.41-5.59 5.59-5.59-5.59-1.41 1.41 5.59 5.59-5.59 5.59 1.41 1.41 5.59-5.59 5.59 5.59 1.41-1.41-5.59-5.59z"></path></g></svg>';
    var addSvg = '<svg height="20" width="20"><g><path d="M19 13h-6v6h-2v-6h-6v-2h6v-6h2v6h6v2z"></path></g></svg>';
    var rightSvg = '<svg height="20" width="20"><g><path d="M12 4l-1.41 1.41 5.58 5.59h-12.17v2h12.17l-5.58 5.59 1.41 1.41 8-8z"></path></g></svg>';
    var leftSvg = '<svg height="20" width="20"><g><path d="M20 11h-12.17l5.59-5.59-1.42-1.41-8 8 8 8 1.41-1.41-5.58-5.59h12.17v-2z"></path></g></svg>';
    var starSvg = '<svg height="20" width="20"><g><path d="M22 9.24l-7.19-.62L12 2 9.19 8.63 2 9.24l5.46 4.73L5.82 21 12 17.27 18.18 21l-1.63-7.03L22 9.24zM12 15.4l-3.76 2.27 1-4.28-3.32-2.88 4.38-.38L12 6.1l1.71 4.04 4.38.38-3.32 2.88 1 4.28L12 15.4z"></path></g></svg>';

    var svg = '';
    switch(icon) {
        case Icon.ADD:
            svg = addSvg;
            break;
        case Icon.CLOSE:
            svg = closeSvg;
            break;
        case Icon.LEFT:
            svg = leftSvg;
            break;
        case Icon.RIGHT:
            svg = rightSvg;
            break;
        case Icon.STAR:
            svg = starSvg;
            break;
    }

    this.icon = null;

    fabric.loadSVGFromString(svg, function(objects, options) {
        thiz.icon = fabric.util.groupSVGElements(objects, options);

        thiz.icon.set({
            left: thiz.container.left - 2,
            top: thiz.container.top - 2,
            originX: 'center',
            originY: 'center',
            hasControls: false,
            hasBorders: false,
            selectable: false
        }).setCoords();

        if(callbacks.length > 0) {
            for(var i in callbacks) {
                if(callbacks.hasOwnProperty(i)) {
                    var callback = callbacks[i];
                    thiz.icon.on('mousedown', function (e) {
                      //Ensure clicking adorners doesn't de-select the object they adorn
                      thiz.canvas.setActiveObject(thiz.target);

                      callback(e);
                    });
                }
            }
        }
    });

    this.isOpen = false;
    this.objectName = name;

    this.bringToFront = function() {
        thiz.container.bringToFront();
        thiz.innercontainer.bringToFront();

        if(thiz.icon) {
            thiz.icon.bringToFront();
        }
    };

    this.updateTarget = function(t) {
        thiz.target = t;
        thiz.refresh(t.left + t.width / 2, t.top + t.height / 2);
    };

    this.clear = function() {
        thiz.isClearing = true;

        thiz.canvas.remove(thiz.container);
        thiz.canvas.remove(thiz.innercontainer);

        if(thiz.icon) {
            thiz.canvas.remove(thiz.icon);
        }

        thiz.isOpen = false;
        thiz.isClearing = false;
    };

    this.show = function() {
        thiz.container.radius = 20;
        thiz.innercontainer.radius = thiz.container.radius - thickness;

        thiz.canvas.add(thiz.container);
        thiz.canvas.add(thiz.innercontainer);

        if(thiz.icon) {
            thiz.canvas.add(thiz.icon);
        }

        thiz.isOpen = false;
    };

    this.refresh = function(x, y) {
        centerX = x;
        centerY = y;

        thiz.container.left = centerX + Math.cos(thiz.angle) * (thiz.radius * 1.1);
        thiz.container.top = centerY + Math.sin(thiz.angle) * (thiz.radius * 1.1);

        thiz.innercontainer.left = thiz.container.left;
        thiz.innercontainer.top = thiz.container.top;

        if(thiz.icon) {
            thiz.icon.left = thiz.container.left - 2;
            thiz.icon.top = thiz.container.top - 2;
        }
    };

    //
    // Manage click events for the adorner items
    //
    this.addListener = function (callback) {
        thiz.container.on('mousedown', function (e) {
            //Ensure clicking adorners doesn't de-select the object they adorn
            thiz.canvas.setActiveObject(thiz.target);

            callback(e);
        });

        thiz.innercontainer.on('mousedown', function (e) {
            //Ensure clicking adorners doesn't de-select the object they adorn
            thiz.canvas.setActiveObject(thiz.target);

            callback(e);
        });

        callbacks.push(callback);

        if(thiz.icon) {
            thiz.icon.on('mousedown', function (e) {
                //Ensure clicking adorners doesn't de-select the object they adorn
                thiz.canvas.setActiveObject(thiz.target);

                callback(e);
            });
        }
    };

    this.dispose = function() {
        thiz.container.off();
        thiz.innercontainer.off();

        if(thiz.icon) {
            thiz.icon.off();
        }

        callbacks = [];
    };

    //
    // Animations of the adorner item
    //
    this.animateAdorner = function() {
        if(!thiz.isOpen) {
            thiz.container.radius = 5;
            thiz.innercontainer.radius = thiz.container.radius - thickness;
            thiz.canvas.add(thiz.container);
            thiz.canvas.add(thiz.innercontainer);

            thiz.container.animate('radius', 20, {
                duration: 300,
                easing: fabric.util.ease.easeOutBounce,
                onChange: function () {
                    thiz.innercontainer.radius = thiz.container.radius - thickness;

                    thiz.canvas.renderAll.bind(thiz.canvas);
                    //thiz.canvas.renderAll();
                },
                onComplete: function() {
                    if(thiz.icon && !thiz.isClearing) {
                        thiz.canvas.add(thiz.icon);
                    }
                },
                abort: function() {
                    return thiz.isClearing;
                }
            });

            thiz.isOpen = true;

        } else {

            thiz.container.radius = 25;
            thiz.innercontainer.radius = thiz.container.radius - thickness;

            if(thiz.icon) {
                thiz.canvas.remove(thiz.icon);
            }

            thiz.container.animate('radius', 5, {
                duration: 300,
                easing: fabric.util.ease.easeOutBounce,
                onChange: function () {
                    thiz.innercontainer.radius = thiz.container.radius - thickness;

                    thiz.canvas.renderAll.bind(thiz.canvas);
                },
                onComplete: function() {
                    thiz.canvas.remove(thiz.innercontainer);
                    thiz.canvas.remove(thiz.container);
                }
            });

            thiz.isOpen = false;
        }
    };

    function selectPlacementAngle(ord, total) {
        var angle = Math.PI;

        //Decide where the adorner should be placed based on the total number of adorners and the position of this adorner
        // item in the "ordered list" of adorners
        if(total === 1) {

            angle = 0;

        } else if(total === 2) {

            if(ord === 1 ) {
                angle *= (1/4);
            } else {
                angle *= (3/4);
            }

        } else if(total === 3) {

            if(ord === 1 ) {
                angle *= (1/4);
            } else if(ord === 2) {
                angle *= (2/4);
            } else if(ord === 3 ) {
                angle *= (3/4);
            }
        } else if(total === 4) {

            if(ord === 1 ) {
                angle *= (1/4);
            } else if(ord === 2) {
                angle *= (2/4);
            } else if(ord === 3 ) {
                angle *= (3/4);
            } else if(ord === 4 ) {
                angle *= (6/4);
            }
        }

        return angle - Math.PI / 2;
    }
}
