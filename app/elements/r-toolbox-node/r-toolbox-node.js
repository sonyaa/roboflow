Polymer('r-toolbox-node', {
    publish: {
        src: '',
        name: '',
        optSet: {},
        preConds: [],
        postConds: [],
        color: '',
        type: '',
        isStart: false,
        isEnd: false,
        isOperation: false
    },

    ready: function() {
        if (this.hasAttribute('start')) {
            this.isStart = true;
            this.type = 'start';
        } else if (this.hasAttribute('end')) {
            this.isEnd = true;
            this.type = 'end';
        } else if (this.hasAttribute('operation')) {
            this.isOperation = true;
            this.type = 'operation';
        }
    },

    attached: function() {
        this.fire('newNodeInToolbox', {target: this});
    }
});
