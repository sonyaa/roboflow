// Author: Alexander Fiannaca

function Adorners(c, t, sc) {
    var thiz = this;

    this.canvas = c;
    this.target = t;
    this.showCircle = sc;

    var isForward = false;

    this.circle = new fabric.Circle({
        radius: Math.sqrt(t.width * t.width + t.height * t.height) / 2 + 12,
        left: t.left + t.width / 2,
        top: t.top + t.height / 2,
        originX: 'center',
        originY: 'center',
        centeredRotation: true,
        angle: -90,
        startAngle: 0,
        endAngle: 0.2,
        stroke: '#000',
        strokeWidth: 4,
        fill: '',
        selectable: false,
        hasControls: false,
        hasBorders: false
    });
    this.circle.isOpen = false;

    this.options = [];

    this.addAdorner = function(name, fill, ord, total, icon, radius) {
        for(var i = 0; i < thiz.options.length; ++i) {
            if(thiz.options[i].objectName === name) {
                console.error('An adorner with this name already exists: ' + name);
                break;
            }
        }

        if(thiz.showCircle) {
            thiz.options.push(new Adorner(thiz.canvas, thiz.target, name, fill, ord, total, icon, radius));
        } else {
            thiz.options.push(new Adorner(thiz.canvas, thiz.target, name, fill, 1, 1, icon, 0));
        }
    };

    this.updateTarget = function(t) {
        thiz.target = t;

        for(var i = 0; i < thiz.options.length; ++i) {
            thiz.options[i].updateTarget(t);
        }
    };

    this.addListener = function(name, func) {
        for(var i = 0; i < thiz.options.length; ++i) {
            if(thiz.options[i].objectName === name) {
                thiz.options[i].addListener(func);
                break;
            }
        }
    };

    this.target.on('moving', function(e) {
        thiz.refresh();
    });

    this.refresh = function() {
        thiz.circle.left = thiz.target.left + thiz.target.width / 2;
        thiz.circle.top = thiz.target.top + thiz.target.height / 2;

        for(var i = 0; i < thiz.options.length; ++i) {
            thiz.options[i].refresh(thiz.circle.left, thiz.circle.top);
        }
    };

    var forward = function(data) {
        thiz.circle.endAngle = 0;

        if(thiz.showCircle) {

            thiz.canvas.add(thiz.circle);

            thiz.circle.animate('endAngle', Math.PI * 2, {
                duration: 500,
                onChange: function () {
                    for (var i = 0; i < thiz.options.length; ++i) {
                        if ((thiz.options[i].animateAngle === 0 || thiz.circle.endAngle > thiz.options[i].animateAngle) && !thiz.options[i].isOpen) {
                            thiz.options[i].animateAdorner();
                        }
                    }

                    thiz.canvas.renderAll();
                },
                onComplete: function () {
                    thiz.canvas.bringToFront(thiz.target);

                    if(thiz.options.length === 5) {
                        thiz.options[4].bringToFront();
                    }

                    thiz.circle.isOpen = true;
                }
            });
        } else {
            for (var i = 0; i < thiz.options.length; ++i) {
                thiz.options[i].animateAdorner();
            }

            thiz.canvas.renderAll();
            isForward = true;
        }
    };

    var backward = function(data) {
        thiz.circle.endAngle = Math.PI * 2;

        if(thiz.showCircle) {
            thiz.circle.animate('endAngle', 0, {
                duration: 500,
                onChange: function () {
                    for (var i = 0; i < thiz.options.length; ++i) {
                        if ((thiz.options[i].animateAngle === 0 || thiz.circle.endAngle < thiz.options[i].animateAngle) && thiz.options[i].isOpen) {
                            thiz.options[i].animateAdorner();
                        }
                    }

                    thiz.canvas.renderAll();
                },
                onComplete: function () {
                    thiz.canvas.remove(thiz.circle);
                    thiz.circle.isOpen = false;
                }
            });
        } else {
            for (var i = 0; i < thiz.options.length; ++i) {
                thiz.options[i].animateAdorner();
            }

            thiz.canvas.renderAll();
            isForward = false;
        }
    };

    this.clear = function() {
        if(thiz.showCircle) {
            thiz.canvas.remove(thiz.circle);
            thiz.circle.isOpen = false;
        }

        for(var i = 0; i < thiz.options.length; ++i) {
            thiz.options[i].clear();
        }
    };

    this.show = function() {
        if(thiz.showCircle) {
            thiz.circle.endAngle = Math.PI * 2;
            thiz.canvas.add(thiz.circle);
            thiz.circle.isOpen = false;
        }

        for(var i = 0; i < thiz.options.length; ++i) {
            thiz.options[i].show();
        }
    };

    var mouseEvents = function (parameters) {
        if (parameters.target && parameters.target === thiz.target) {

            if(parameters.e.which === MouseButton.RIGHT) {

                if((thiz.showCircle && !thiz.circle.isOpen) || (!thiz.showCircle && !isForward)) {
                    thiz.canvas.setActiveObject(thiz.target);
                    forward(parameters);
                } else {
                    backward(parameters);
                }

            }

        } else {
            thiz.clear();
        }
    };

    this.canvas.on('mouse:down', mouseEvents);

    this.dispose = function() {
        thiz.clear();

        thiz.canvas.off('mouse:down', mouseEvents);

        for(var i = 0; i < thiz.options.length; ++i) {
            thiz.options[i].dispose();
        }
    };
}
