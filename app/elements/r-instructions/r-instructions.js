Polymer('r-instructions', {
    page: 0,

    toggle: function() {
        var sel = null;

        if (this.page === 0) {
            sel = $('r-instructions');
            sel.next().after(sel);

            this.page = 1;
        } else {
            sel = $('r-instructions');
            sel.prev().before(sel);

            this.page = 0;
        }
    }
});
