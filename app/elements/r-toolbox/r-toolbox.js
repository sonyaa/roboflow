(function() {
    var roboflowCanvas = null;
    var nodeToAdd = null;
    var nextId = 0;

    Polymer('r-toolbox', {
        domReady: function() {
            var thiz = this;

            roboflowCanvas = document.querySelector('r-interface');
        },

        dragStart: function (e) {
            var dragInfo = e.detail;
            var color = dragInfo.event.target.style.backgroundColor;

            dragInfo.avatar.style.cssText = 'border: 3px solid ' + color + '; width: 32px; height: 32px; border-radius: 32px; background-color: whitesmoke';
            dragInfo.drag = function() {};
            dragInfo.drop = this.drop;
        },

        drop: function(dragInfo) {
            var dropTarget = dragInfo.event.relatedTarget;
            var f = dragInfo.framed;
            var node = dragInfo.event.target;

            if(dropTarget.tagName.toLowerCase() === 'canvas') {
                node.id = nextId;
                if (node.type && node.type === NodeType.OPERATION) {
                    roboflowCanvas.addOperation(f.x, f.y, node);
                } else if (node.type && node.type === NodeType.START) {
                    roboflowCanvas.addStart(f.x, f.y, node);
                }else if (node.type && (node.type === NodeType.END_SUCCESS || node.type === NodeType.END_FAIL)) {
                    roboflowCanvas.addEnd(f.x, f.y, node);
                }
                nextId ++;
            }
        }
    });
})();
