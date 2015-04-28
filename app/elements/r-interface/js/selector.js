// Author: Alexander Fiannaca

function Selector(c, t) {
    var thiz = this;
    var canvas = c;
    var isSelected = false;
    var target = t;

    // Create a static count variable to track how many instances of this we have
    if ( typeof Selector.initCount === 'undefined' ) {
        // It has not... perform the initialization
        Selector.initCount = 0;
    }

    //
    // Callbacks
    //

    var remove = function() {
        canvas.remove(thiz.outline);
        isSelected = false;
    };

    var targetSelected = function(parameters) {
        isSelected = true;

        thiz.outline.width = target.width + 7;
        thiz.outline.height = target.height + 7;

        thiz.outline.top = target.top - 5;
        thiz.outline.left = target.left - 5;

        canvas.add(thiz.outline);

        canvas.bringToFront(target);
    };

    var targetMoving = function(parameters) {
        thiz.outline.top = target.top - 5;
        thiz.outline.left = target.left - 5;
    };

    this.refresh = function() {
        if(isSelected) {
            thiz.outline.top = target.top - 5;
            thiz.outline.left = target.left - 5;
        }
    };

    //
    // Setup the callbacks
    //

    if(Selector.initCount === 0) {
        canvas.on('object:selected', remove);
        canvas.on('selection:cleared', remove);
    }

    target.on('selected', targetSelected);
    target.on('moving', targetMoving);

    //
    // Cleanup when the object is dying
    //

    this.dispose = function() {
        Selector.initCount -= 1;

        if(Selector.initCount === 0) {
            canvas.off('object:selected', remove);
            canvas.off('selection:cleared', remove);

        }

        target.off('selected', targetSelected);
        target.off('moving', targetMoving);

        if(isSelected) {
            thiz.outline.left = -500;
            thiz.outline.top = -500;
            thiz.outline.setCoords();
        }
    };

    // Track the number of instances of Selector which have been created
    Selector.initCount += 1;
}

// All of the selector objects share this outline object
Selector.prototype.outline = new fabric.Rect({
    left: -500,
    top: -500,
    fill: '',
    stroke: 'rgb(0,192,255)',
    strokeWidth: 3,
    selectable: false
});
