// Author: Alexander Fiannaca
// Author: Sonya Alexandrova

(function() {
    var canvas = null;

    var deletingItem = null;

    function sizeCanvas(h, v) {
        var width = window.innerWidth - h - 40;
        var height = window.innerHeight - v - 70;

        canvas.setHeight(height);
        canvas.setWidth(width);

        canvas.forEachObject(function(object) {
            if(object.isNode) {
                setBounds(object);
                updatePosition(object);

                //We must call setCoords() and manually fire the moving event in order to properly update the
                // interface!
                object.setCoords();
                object.fire('moving');
            }
            if(object.isPlug || object.isSocket) {
                //We must call setCoords() and manually fire the moving event in order to properly update the
                // interface!
                object.setCoords();
                object.fire('moving');
            }
        });

        canvas.renderAll();
    }

    function setBounds(node) {
        node.yMin = 0;
        node.yMax = canvas.height;
        node.xMin = 0;
        node.xMax = canvas.width;
    }

    function updatePosition(node) {
        node.left = canvas.width * node.xPercent - (node.width / 2);
        node.selector.refresh();
        node.adorners.refresh();
        if (node.hasOwnProperty('socket') && node.socket !== null) {
            node.socket.refresh();
        }
        if (node.hasOwnProperty('plugs')) {
            node.plugs.forEach(function (plug) {
                plug.refresh();
        });
    }
    }

    function isOverlapping(s1, e1, s2, e2) {
        return !(e2 < s1 || e1 < s2);
    }

    function setInitialXY(x, y, type) {
        var ret = {};
        ret.x = x;
        ret.y = y;

        //Make sure the initial x and y values are inside the boundaries for each section
        if(ret.y < 0 ) {
            ret.y = 0;
        } else if(ret.y > canvas.height) {
            ret.x = canvas.height - 100;
        }

        var tmp = 0;
        if(ret.x === 0 && ret.y === 0) {
            ret.x = (canvas.width / 2) - 50;
            ret.y = 50;
        }

        //Make sure that the bounding box for the node is not overlapping with a pre-existing node
        var occupied = [];
        canvas.forEachObject(function(object) {
            if(object.type && object.type === type) {
                occupied.push({
                    startX: object.left,
                    endX: object.left + object.width,
                    startY: object.top,
                    endY: object.top + object.height
                });
            }
        });

        occupied.sort(function(a,b) {
            if(a.startY === b.startY) {
                return a.startX - b.startX;
            }

            return a.startY - b.startY;
        });

        for(var i = 0; i < occupied.length; i++) {
            if(isOverlapping(occupied[i].startX, occupied[i].endX, ret.x, ret.x + 100)) {

                if (isOverlapping(occupied[i].startY, occupied[i].endY, ret.y, ret.y + 100)) {
                    ret.y = occupied[i].endY + 40;

                    // If the node is still overlapping, try moving to the right
                    if(isOverlapping(occupied[i].startY, occupied[i].endY, ret.y, ret.y + 100)) {
                        ret.x = occupied[i].endX + 40;
                    }
                }
            }

            if (i === occupied.length - 1 && ret.y + 80 > canvas.height) {
                ret.y = canvas.height - 80;
            }
        }

        return ret;
    }

    //
    //function isTransformComplete(node) {
    //    var objects = canvas.getObjects();
    //
    //    for(var i = 0, l = objects.length; i < l; ++i) {
    //        if(objects[i].isNode && !objects[i].isComplete) {
    //            return false;
    //        }
    //    }
    //
    //    return true;
    //}

    Polymer('r-interface', {

        publish: {
            horizontalOffset: '256px',
            verticalOffset: '65px',
            errorMsg: '',
            infoMsg: '',
            current: {},
            optSet: [],
            actions: [],
            action_id: null,
            action_name: null,
            optionSetDict: {},
            executionServices: {},
            executionTimeouts: {},
            preconditionServices: {}
        },

        observe: {
            'optSet.options[0].value': 'processOptions'
        },

        OptionType: OptionType,

        NodeType: NodeType,

        ready: function() {
            //
            // Create the canvas
            //
            canvas = new fabric.Canvas(this.$.c);

            //Make sure that the node stay inside of the interface
            canvas.on('object:moving', function(params) {
                var object = params.target;

                //Check if the moved object is a node in the graph
                if(object.isNode) {
                    if (object.top < object.yMin) {
                        object.top = object.yMin;
                    } else if (object.top + object.height > object.yMax) {
                        object.top = object.yMax - object.height;
                    }

                    if (object.left < object.xMin) {
                        object.left = object.xMin;
                    } else if (object.left + object.width > object.xMax) {
                        object.left = object.xMax - object.width;
                    }
                }
            });

            //
            // Handle window resizes by properly sizing the canvas
            //
            var thiz = this;
            //window.addEventListener('resize', function() {
            //    sizeCanvas(parseInt(thiz.horizontalOffset,10), parseInt(thiz.verticalOffset,10));
            //});

            sizeCanvas(parseInt(this.horizontalOffset,10), parseInt(this.verticalOffset,10));

            //
            // Prevent the right click context menu
            //
            $(document).bind('contextmenu', function(e) {
                return false;
            });

        },

        getCanvas: function() {
            return canvas;
        },

        restart: function() {
            canvas.forEachObject(function(object) {
                if(object && object.isNode) {
                    //for(var i = object.edges.outgoing.length - 1; i >= 0; i--) {
                    //    object.edges.outgoing[i].dispose();
                    //}
                    //
                    //for(var j = object.edges.incoming.length - 1; j >= 0; j--) {
                    //    object.edges.incoming[j].dispose();
                    //}

                    object.adorners.dispose();
                    object.selector.dispose();
                    if (object.hasOwnProperty('socket') && object.socket !== null) {
                        object.socket.dispose();
                    }
                    if (object.hasOwnProperty('plugs')) {
                        object.plugs.forEach(function (plug) {
                            plug.dispose();
                        });
                    }
                }
            });

            canvas.clear();
        },

        cancelDelete: function() {
            deletingItem = null;
        },

        confirmDelete: function() {
            canvas.remove(deletingItem);

            //for(var i = deletingItem.edges.outgoing.length - 1; i >= 0; --i) {
            //    deletingItem.edges.outgoing[i].dispose();
            //}
            //
            //for(var j = deletingItem.edges.incoming.length - 1; j >= 0; --j) {
            //    deletingItem.edges.incoming[j].dispose();
            //}

            deletingItem.adorners.dispose();
            deletingItem.selector.dispose();
            if (deletingItem.hasOwnProperty('socket') && deletingItem.socket !== null) {
                deletingItem.socket.dispose();
            }
            if (deletingItem.hasOwnProperty('plugs')) {
                deletingItem.plugs.forEach(function (plug) {
                    plug.dispose();
                });
            }
            deletingItem = null;
        },

        error: function(msg, isHTML) {
            $(this.$.errorMsg).empty();

            if(typeof isHTML !== 'undefined' && isHTML) {
                this.injectBoundHTML(msg, this.$.errorMsg);
            } else {
                this.injectBoundHTML('{{errorMsg}}', this.$.errorMsg);
            }

            this.errorMsg = msg;
            this.$.errorDialog.toggle();
        },

        clearError: function() {
            this.errorMsg = '';
        },

        info: function(msg) {
            $(this.$.infoMsg).empty();
            this.infoMsg = msg;
            this.injectBoundHTML('{{infoMsg}}', this.$.infoMsg);
            this.$.infoDialog.toggle();
        },

        setOptionsView: function(node, optSet) {
            this.current = node;
            this.optSet = optSet;

            return (this.optSet.options.length > 0);
        },

        showOptionsView: function(node, preventToggle) {
            if(typeof preventToggle === 'undefined') {
                preventToggle = false;
            }
            if (node.type === NodeType.OPERATION) {
                if (node.optSet) {
                    if (this.setOptionsView(node, node.optSet)) {
                        this.$.optsDialog.toggle();
                        return;
                    }
                }
                this.error('There are no options to set for this item!');
            }
        },

        processOptions: function(oldValue, newValue) {
            if (newValue >= 0) {
                // items[i]: [name, id]
                this.current.step_id = this.optSet.options[0].items[newValue][1];
                var step_name = this.optSet.options[0].items[newValue][0];
                this.updateOperationStepName(this.current, step_name);
                this.current.canvas.renderAll();
            }
        },

        updateOperationStepName: function(node, step_name) {
            if (!step_name) {
                step_name = 'null';
                var optItems = this.optionSetDict[node.operationType].options[0].items;
                for (var i = 0; i < optItems.length; i++) {
                    if (optItems[i][1] == node.step_id) {
                        step_name = optItems[i][0];
                        break;
                    }
                }
            }
            node.item(0).item(0).set({'text':node.name + '\n(' + step_name + ')'});
        },

        processNewNode:function(node, type, tool, x, y) {

            node.isNode = true;
            node.isComplete = false;
            node.type = type;

            node.edges = {};
            node.edges.outgoing = [];
            node.edges.incoming = [];

            node.name = tool.name;
            node.color = tool.color;
            node.id = tool.id;

            node.step_id = null;
            node.xPercent = (node.left + (node.width / 2)) / canvas.width;
            node.on('moving', function(e) {
                node.xPercent = (node.left + (node.width / 2)) / canvas.width;
            });

            node.updateState = function () {
                console.log('Updating state of ' + node.name);
                // TODO
                node.isComplete = true;
            };

            if(type === NodeType.OPERATION) {
                node.operationType = tool.operationType;
                if (tool.optSet && tool.optSet.hasOptions) {
                    node.optSet = tool.optSet.clone();
                }
            }

            //Define where the node can be in the interface
            setBounds(node);

            node.selector = new Selector(canvas, node);
            node.adorners = new Adorners(canvas, node, true);

            var deleteDialog = this.$.deleteDialog;

            node.adorners.addAdorner('close', 'red', 1, 2, Icon.CLOSE);
            node.adorners.addListener('close', function (e) {
                deletingItem = node;
                deleteDialog.toggle();
            });


            return node;
        },

        addOperation: function(x, y, nodeTool) {
            var self = this;

            var pos = setInitialXY(x, y, NodeType.OPERATION);

            var node = buildOperationNode(nodeTool, pos.x, pos.y, canvas);
            this.processNewNode(node, NodeType.OPERATION, nodeTool, pos.x, pos.y);

            //Handle options interface
            node.on('dblclick', function() {
                self.showOptionsView(node);
            });

            canvas.add(node);

            return node;
        },

        addStart: function(x, y, nodeTool) {
            var self = this;

            var pos = setInitialXY(x, y, NodeType.START);

            var node = buildStartNode(nodeTool, pos.x, pos.y, canvas);
            this.processNewNode(node, NodeType.START, nodeTool, pos.x, pos.y);

            canvas.add(node);

            return node;
        },

        addEnd: function(x, y, nodeTool) {
            var self = this;

            var pos = setInitialXY(x, y, nodeTool.type);

            var node = buildEndNode(nodeTool, pos.x, pos.y, canvas);
            this.processNewNode(node, nodeTool.type, nodeTool, pos.x, pos.y);

            canvas.add(node);

            return node;
        },

        validateGraph: function() {
            if (!areAllPlugsUsed(canvas)) {
                this.error("All plugs must be connected to sockets.");
                return null;
            }
            var graph = getGraphFromCanvas(canvas);
            var error_msg = getGraphValidationError(graph);
            if (error_msg !== null) {
                this.error(error_msg);
                return null;
            } else {
                //this.info("Graph is valid!");
                return graph;
            }
        },

        executeNode: function(node, graph) {
            if (node.type == NodeType.END_SUCCESS) {
                this.info('Action completed successfully!');
            } else if (node.type == NodeType.END_FAIL) {
                this.info('Action failed.');
            } else if (node.type == NodeType.OPERATION) {
                if (node.preConds.length == 0) {
                    this.executeOperationAndPostconditions(node, graph);
                } else {
                    this.executePrecondition(node, graph, 0);
                }
            } else {
                console.warn('Invalid node type: ' + node.type);
            }
        },

        waitFor: function(test, msec, count, callback, timeOut) {
            var that = this;
            // Check if we got a response from server. If not, re-check later (msec).
            if (!test()) {
                count++;
                if (count*msec/1000 > timeOut) {
                    this.error('Timeout while waiting for execution result; action aborted!');
                    return;

                }
                setTimeout(function() {
                    that.waitFor(test, msec, count, callback, timeOut);
                }, msec);
                return;
            }
            // Condition finally met. callback() can be executed.
            console.log('Condition met! Will execute callback.');
            callback();
        },

        executeOperationAndPostconditions: function(node, graph) {
            var that = this;
            var timeOut = this.executionTimeouts[node.operationType];
            if (!timeOut) {
                timeOut = 30;
            }
            var gotResponse = false;
            this.executionServices[node.operationType].callService(new ROSLIB.ServiceRequest({
                'step_id': node.step_id
            }), function (result) {
                gotResponse = true;
                console.log('Received execution status: ' + result.status);
                node.status = result.status;
            });

            var test = function() {
                return gotResponse;
            };
            var gotStatusCb = function() {
                //This gets called after we receive the execution status from the server.
                if (node.postConds.length == 0) {
                    // Node has no postconditions - exit to the first target after preconditions.
                    that.executeNode(graph.vertices[node.targets[node.preConds.length]], graph);
                } else {
                    for (var i = 0; i < node.postConds.length; i++) {
                        var postCond = node.postConds[i];
                        // If postondition is not satisfied, exit to corresponding target, otherwise continue to next condition.
                        if (!checkPostCondition(postCond, node.status)) {
                            console.log('Post condition "' + postCond + '" failed.');
                            that.executeNode(graph.vertices[node.targets[node.preConds.length+i]], graph);
                            return;
                        }
                    }
                    // If we got to here, that means all postconditions were satisfied. Exit to last target.
                    // At this point i == number of postconditions == index of last target.
                    console.log('All postconditions were successful.');
                    that.executeNode(graph.vertices[node.targets[node.preConds.length+i]], graph);
                    node.status = null;
                }
            };

            this.waitFor(test, 500, 0, gotStatusCb, timeOut);
        },

        executePrecondition: function(node, graph, preCondIndex) {
            var that = this;
            var precondTimeout = 10;
            var gotResponse = false;
            var condition = node.preConds[preCondIndex];
            var preCondResult = false;
            this.preconditionServices[node.operationType][condition].callService(new ROSLIB.ServiceRequest({
                'step_id': node.step_id
            }), function (result) {
                gotResponse = true;
                console.log('Received precondition check result: ' + result.status + ' for precondition "' + condition + '"');
                preCondResult = result.status;
            });
            var test = function() {
                return gotResponse;
            };
            var gotStatusCb = function() {
                //This gets called after we receive the precondition check response from the server.
                if (!preCondResult) {
                    // If precondition returned false, exit to the corresponding target.
                    that.executeNode(graph.vertices[node.targets[preCondIndex]], graph);
                } else if (preCondIndex+1 < node.preConds.length) {
                    // If there are more conditions continue to the next condition.
                    that.executePrecondition(node, graph, preCondIndex+1);
                } else {
                    // Otherwise execute the node itself.
                    that.executeOperationAndPostconditions(node, graph);
                }
            };

            this.waitFor(test, 500, 0, gotStatusCb, precondTimeout);
        },

        executeGraph: function() {
            var graph = this.validateGraph();
            if (graph != null) {
                // If we're here, it means graph is valid: one start, all nodes connected etc.
                var curVertexId = graph.starts[0].targets[0];
                this.executeNode(graph.vertices[curVertexId], graph);
            }
        },

        getActionData: function() {
            var graph = getGraphFromCanvas(canvas);
            if (this.action_id == null) {
                var id = 0;
                for (var i = 0; i < this.actions.length; i++) {
                    if (this.actions[i].id >= id) {
                        id = this.actions[i].id + 1;
                    }
                }
                this.action_id = id;
            }
            graph.id = this.action_id;
            graph.name = this.action_name;
            return [graph.id, JSON.stringify(graph)];
        },

        loadAction: function(action_id) {
            var new_action = null;
            for (var i = 0; i < this.actions.length; i++) {
                if (this.actions[i].id == action_id) {
                    new_action = this.actions[i];
                    break;
                }
            }
            if (new_action == null) {
                console.warn("Couldn't find action with id " + action_id);
                return;
            }
            console.log("Loading action with id " + action_id);
            this.restart();
            this.showActionInCanvas(new_action);
        },

        showActionInCanvas: function(graph) {
            var tool;
            var node;
            var nodesDict = {};
            var i;
            for (i = 0; i < graph.starts.length; i++) {
                tool = graph.starts[i];
                tool.type = NodeType.START;
                node = this.addStart(tool.x, tool.y, tool);
                node.targets = tool.targets;
                nodesDict[node.id] = node;
            }
            for (i = 0; i < graph.fail_ends.length; i++) {
                tool = graph.fail_ends[i];
                tool.type = NodeType.END_FAIL;
                node = this.addEnd(tool.x, tool.y, tool);
                node.targets = tool.targets;
                nodesDict[node.id] = node;
            }
            for (i = 0; i < graph.success_ends.length; i++) {
                tool = graph.success_ends[i];
                tool.type = NodeType.END_SUCCESS;
                node = this.addEnd(tool.x, tool.y, tool);
                node.targets = tool.targets;
                nodesDict[node.id] = node;
            }
            for (i = 0; i < graph.operations.length; i++) {
                tool = graph.operations[i];
                tool.type = NodeType.OPERATION;
                tool.optSet = this.optionSetDict[tool.operationType];
                node = this.addOperation(tool.x, tool.y, tool);
                node.targets = tool.targets;
                node.step_id = tool.step_id;
                node.operationType = tool.operationType;
                this.updateOperationStepName(node);
                nodesDict[node.id] = node;
            }
            // add edges
            canvas.forEachObject(function(object) {
                var vertex;
                if (object && object.isNode) {
                    for (var i = 0; i < object.targets.length; i++) {
                        if (object.targets[i] != null) {
                            var socket = nodesDict[object.targets[i]].socket;
                            var edge = new Edge(canvas, object.plugs[i]);
                            edge.connect(socket);
                        }
                    }
                }
            });
        }

    });
})();
