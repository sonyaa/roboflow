function buildStartNode(tool, x, y, canvas) {
    var terminalHeight = 30;
    var terminalWidth = 65;
    var linkLength = 20;
    var plugRadius = 10;
    var points = [
        {x: 0, y:terminalHeight},
        {x: terminalWidth - terminalHeight/2, y:terminalHeight},
        {x: terminalWidth, y: terminalHeight / 2},
        {x: terminalWidth - terminalHeight/2, y: 0},
        {x: 0, y: 0}
    ];
    var polygon = new fabric.Polygon(points, {
        fill: tool.color,
        opacity: 0.7,
        originX: 'center',
        originY: 'center'
    });
    var text = new fabric.Text(tool.name, {
        fontSize: 14,
        fill: 'black',
        originX: 'center',
        originY: 'center',
        left: - terminalHeight/2 + linkLength/2
    });
    var line = new fabric.Line([0, 0, linkLength, 0], {
        stroke: tool.color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: 0,
        left: terminalWidth / 2 + linkLength / 2
    });
    var node = new fabric.Group([polygon, text, line], {
        left: x,
        top: y,
        hasBorders: false,
        hasControls: false
    });

    var plug = new Plug(canvas, node, "start", tool.color, plugRadius, (terminalWidth+linkLength)/2+plugRadius, 0);
    node.plugs = [plug];
    node.targets = [null];

    return node;
}

function buildEndNode(tool, x, y, canvas) {
    var terminalHeight = 30;
    var terminalWidth = 65;
    var linkLength = 20;
    var socketRadius = 10;
    var points = [
        {x: 0, y:terminalHeight/2},
        {x: terminalHeight/2, y:terminalHeight},
        {x: terminalWidth, y: terminalHeight},
        {x: terminalWidth, y: 0},
        {x: terminalHeight/2, y: 0}
    ];
    var polygon = new fabric.Polygon(points, {
        fill: tool.color,
        opacity: 0.7,
        originX: 'center',
        originY: 'center'
    });
    var text = new fabric.Text(tool.name, {
        fontSize: 14,
        fill: 'black',
        originX: 'center',
        originY: 'center',
        left: terminalHeight/2 - linkLength/2
    });
    var line = new fabric.Line([0, 0, linkLength, 0], {
        stroke: tool.color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: 0,
        left: - terminalWidth / 2 - linkLength / 2
    });
    var node = new fabric.Group([polygon, text, line], {
        left: x,
        top: y,
        hasBorders: false,
        hasControls: false
    });
    node.socket = getSocket(canvas, node, "end", tool.color, socketRadius, -(terminalWidth+linkLength)/2-socketRadius, 0);
    node.plugs = [];
    return node;
}

function buildOperationNode(tool, x, y, canvas) {
    var tokenHeight = 40;
    var tokenWidth = 100;
    var linkLength = 20;
    var socketRadius = 10;
    var plugRadius = 10;
    var points = [
        {x: 0, y:0},
        {x: 0, y:tokenHeight},
        {x: tokenWidth, y: tokenHeight},
        {x: tokenWidth, y: 0}
    ];
    var polygon = new fabric.Polygon(points, {
        stroke: tool.color,
        strokeWidth: 2,
        fill: 'transparent',
        opacity: 0.7,
        originX: 'center',
        originY: 'center',
        top: 0
    });
    var text = new fabric.Text(tool.name + '\n(null)', {
        fontSize: 14,
        textAlign: 'center',
        fill: 'black',
        originX: 'center',
        originY: 'center',
        left: 0
    });
    var line1 = new fabric.Line([0, 0, 0, linkLength], {
        stroke: tool.color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: - tokenHeight / 2 - linkLength / 2,
        left: 0
    });
    var token = new fabric.Group([text, polygon, line1], {
        left: 0,
        top: 0,
        originX: 'center',
        originY: 'center',
        hasBorders: false,
        hasControls: false
    });
    var elements = [token];
    var conditionHeight = tokenHeight + linkLength;
    var numPreCond = 0;
    for (var i in tool.preConds) {
        if(tool.preConds.hasOwnProperty(i)) {
            elements.push(buildCondition(tool.preConds[i], tool.color,
                tokenHeight, tokenWidth, linkLength, - conditionHeight * i - (tokenHeight + linkLength + conditionHeight)/2));
            numPreCond += 1;
        }
    }
    var numPostCond = 0;
    for (var j in tool.postConds) {
        if(tool.postConds.hasOwnProperty(j)) {
            elements.push(buildCondition(tool.postConds[j], tool.color,
                tokenHeight, tokenWidth, linkLength, conditionHeight * j + (tokenHeight + linkLength + conditionHeight)/2));
            numPostCond += 1;
        }
    }
    var line2 = new fabric.Line([0, 0, 0, linkLength], {
        stroke: tool.color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: tokenHeight / 2 + linkLength + conditionHeight*numPostCond,
        left: 0
    });
    elements.push(line2);
    var node = new fabric.Group(elements, {
        left: x,
        top: y,
        hasBorders: false,
        hasControls: false
    });
    node.socket = getSocket(canvas, node, tool.name, tool.color,
        socketRadius,
        numPostCond+numPreCond > 0 ? -linkLength/2 : 0,
        -(tokenHeight + 2*linkLength + conditionHeight*numPreCond + conditionHeight*numPostCond)/2 - socketRadius);
    node.plugs = [];
    node.targets = [];
    for (i in tool.preConds) {
        if(tool.preConds.hasOwnProperty(i)) {
            node.plugs.push(new Plug(canvas, node, tool.name, tool.color,
                plugRadius,
                node.width/2 + plugRadius,
                - conditionHeight * i - (tokenHeight + linkLength)/2 ));
        }
        node.targets.push(null)
    }
    for (j in tool.postConds) {
        if(tool.postConds.hasOwnProperty(j)) {
            node.plugs.push(new Plug(canvas, node, tool.name, tool.color,
                plugRadius,
                node.width/2 + plugRadius,
                conditionHeight * j + (tokenHeight + linkLength)/2 + conditionHeight*numPreCond/2));
        }
        node.targets.push(null)
    }
    node.plugs.push(new Plug(canvas, node, tool.name, tool.color,
        plugRadius,
        numPostCond+numPreCond > 0 ? -linkLength/2 : 0,
        node.height / 2 + plugRadius));
    node.preConds = tool.preConds;
    node.postConds = tool.postConds;
    return node;
}

function buildCondition(name, color, tokenHeight, tokenWidth, linkLength, yOffset) {
    var points = [
        {x: - tokenWidth / 2, y: 0},
        {x: 0, y: tokenHeight / 2},
        {x: tokenWidth / 2, y: 0},
        {x: 0, y: - tokenHeight / 2}
    ];
    var polygon = new fabric.Polygon(points, {
        stroke: color,
        strokeWidth: 2,
        fill: 'transparent',
        originX: 'center',
        originY: 'center',
        top: 0,
        left: 0
    });
    var text = new fabric.Text(name, {
        fontSize: 14,
        fill: 'black',
        originX: 'center',
        originY: 'center',
        left: 0,
        top: 0
    });
    var line1 = new fabric.Line([0, 0, 0, linkLength], {
        stroke: color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: - tokenHeight / 2 - linkLength / 2,
        left: 0
    });
    var line2 = new fabric.Line([0, 0, 20, 0], {
        stroke: color,
        strokeWidth: 2,
        originX: 'center',
        originY: 'center',
        top: 0,
        left: tokenWidth / 2 + linkLength / 2
    });
    return new fabric.Group([polygon, text, line1, line2], {
        left: linkLength / 2,
        top: yOffset,
        hasBorders: false,
        hasControls: false,
        originX: 'center',
        originY: 'center'
    });
}