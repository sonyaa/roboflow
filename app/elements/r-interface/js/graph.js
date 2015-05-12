function StartVertex(x,y,name,id,color,target) {
    this.x = x;
    this.y = y;
    this.name = name;
    this.id = id;
    this.color = color;
    this.targets = [target];
}

function EndVertex(x,y,name,id,color) {
    this.x = x;
    this.y = y;
    this.name = name;
    this.id = id;
    this.color = color;
    this.targets = [];
}

function OperationVertex(x,y,name,id,color,preConds,postConds,targets,step,type) {
    this.x = x;
    this.y = y;
    this.name = name;
    this.id = id;
    this.color = color;
    this.preConds = preConds;
    this.postConds = postConds;
    this.targets = targets;
    this.step_id = step;
    this.operationType = type;
}

function areAllPlugsUsed(canvas) {
    var isValid = true;
    canvas.forEachObject(function(object) {
        if(object && object.isNode) {
            for (var i = 0; i < object.plugs.length; i++) {
                var plug = object.plugs[i];
                if (plug.edge == null) {
                    isValid = false;
                    return;
                }
            }
        }
    });
    return isValid;
}

function getGraphFromCanvas(canvas) {
    var graph = Object();
    //graph.start, graph.success_ends, graph.fail_ends, graph.operations
    //op.name, op.id, op.preconds, op.postconds, op.targets, op.color, op.x, op.y
    //start.x, start.y, start.target, start.name, start.color
    //end.x, end.y, end.color, end.name
    graph.starts = [];
    graph.success_ends = [];
    graph.fail_ends = [];
    graph.operations = [];
    graph.vertices = {};

    canvas.forEachObject(function(object) {
        var vertex;
        if(object && object.isNode) {
            var target = null;
            if (object.type == NodeType.START) {
                if (object.plugs[0].edge != null) {
                    target = object.plugs[0].edge.to.node.id;
                }
                vertex = new StartVertex(object.left, object.top, object.name, object.id, object.color,
                                        target);
                graph.vertices[object.id] = vertex;
                graph.starts.push(vertex);
            } else if (object.type == NodeType.END_FAIL) {
                vertex = new EndVertex(object.left, object.top, object.name, object.id, object.color);
                graph.vertices[object.id] = vertex;
                graph.fail_ends.push(vertex);
            } else if (object.type == NodeType.END_SUCCESS) {
                vertex = new EndVertex(object.left, object.top, object.name, object.id, object.color);
                graph.vertices[object.id] = vertex;
                graph.success_ends.push(vertex);
            } else if (object.type == NodeType.OPERATION) {
                var targets = [];
                for (var i = 0; i < object.plugs.length; i++) {
                    var plug = object.plugs[i];
                    target = null;
                    if (plug.edge != null) {
                        target = plug.edge.to.node.id;
                    }
                    targets.push(target);
                }
                vertex = new OperationVertex(object.left, object.top, object.name, object.id, object.color,
                                            object.preConds, object.postConds, targets,
                                            object.step_id, object.operationType);
                graph.vertices[object.id] = vertex;
                graph.operations.push(vertex);
            } else {
                console.warn("Unknown node type")
            }
            vertex.type = object.type;
        }
    });
    return graph;
}

function isGraphConnected(graph) {
    var visited = [];
    var stack = [];
    if (graph.starts.length == 0 || graph.starts[0].targets.length == 0 || graph.starts[0].targets[0] == null) {
        // Graph is invalid.
        console.warn('Graph is invalid: start either is absent or is not connected to anything.');
        return false;
    }
    visited[graph.starts[0].id] = true;
    stack.push(graph.starts[0].targets[0]);
    while (stack.length > 0) {
        var nodeId = stack.pop();
        visited[nodeId] = true;
        for (var i = 0; i < graph.vertices[nodeId].targets.length; i++) {
            var targetId = graph.vertices[nodeId].targets[i];
            if (targetId == null) {
                // Graph is invalid.
                console.warn('Graph is invalid: one of the plugs in not connected to anything.');
                return false;
            }
            if ((targetId in graph.vertices) && !(targetId in visited)) {
                stack.push(targetId);
            }
        }
    }
    // Check that we visited all vertices.
    for (var v in graph.vertices) {
        if (graph.vertices.hasOwnProperty(v)) {
            if (!(v in visited)) {
                return false;
            }
        }
    }
    return true;
}

function getGraphValidationError(graph) {
    if (graph.starts.length == 0 || graph.starts.length > 1) {
        return "A valid program must have exactly one start terminal.";
    }
    if (graph.success_ends == 0) {
        return "A valid program must have at least one success end terminal.";
    }
    if (!isGraphConnected(graph)) {
        return "All tokens must be connected."
    }
    return null;
}