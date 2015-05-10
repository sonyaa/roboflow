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
        isOperation: false,
        operationType: []
    },

    ready: function() {
        if (this.hasAttribute('start')) {
            this.isStart = true;
            this.type = 'start';
        } else if (this.hasAttribute('end_success')) {
            this.isEnd = true;
            this.type = 'end_success';
        } else if (this.hasAttribute('end_fail')) {
            this.isEnd = true;
            this.type = 'end_fail';
        } else if (this.hasAttribute('operationType')) {
            this.isOperation = true;
            this.type = 'operation';
        }
    },

    attached: function() {
        this.fire('newNodeInToolbox', {target: this});
    }
});
