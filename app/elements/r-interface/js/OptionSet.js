// Author: Alexander Fiannaca

var OptionType = {
    STRING: 'string',
    INT: 'int',
    DROPDOWN: 'dropdown',
    SLIDER: 'slider'
};

function OptionSet() {

    this.options = [];

    this.hasOptions = false;

    var defaults = {};
    defaults[OptionType.STRING] = '';
    defaults[OptionType.INT] = 0;
    defaults[OptionType.DROPDOWN] = [''];
    defaults[OptionType.SLIDER] = 0;

    this.addString = function(label, key, alt, value) {
        this.hasOptions = true;

        if(typeof value === 'undefined') {
            value = defaults[OptionType.STRING];
        }

        this.options.push({
            type: OptionType.STRING,
            key: key,
            alt: alt,
            value: value
        });
    };

    this.addInt = function(label, key, alt, value) {
        this.hasOptions = true;

        if(typeof value === 'undefined') {
            value = defaults[OptionType.INT];
        }

        this.options.push({
            type: OptionType.INT,
            key: key,
            alt: alt,
            value: value
        });
    };

    this.addDropDown = function(label, key, alt, list, value) {
        this.hasOptions = true;

        if(typeof list === 'undefined') {
            list = defaults[OptionType.DROPDOWN];
        }

        if(typeof value === 'undefined') {
            value = -1;
        }

        this.options.push({
            type: OptionType.DROPDOWN,
            label: label,
            key: key,
            alt: alt,
            value: value,
            items: list
        });
    };

    this.addSlider = function(key, alt, min, max, value) {
        this.hasOptions = true;

        if(typeof value === 'undefined') {
            value = defaults[OptionType.SLIDER];
        }

        this.options.push({
            type: OptionType.SLIDER,
            key: key,
            alt: alt,
            max: max,
            min: min,
            value: value
        });
    };

    this.addValidator = function(key, validator) {
        for(var i = 0, l = this.options.length; i < l; ++i) {
            if(this.options[i].key === key) {
                this.options[i].validator = validator;
                break;
            }
        }
    };

    this.getTemplateList = function(type) {
        var ret = [];

        for(var i = 0, l = this.options.length; i < l; ++i) {
            if(this.options[i].type === type) {
                ret.push(this.options[i]);
            }
        }

        return ret;
    };

    this.clone = function() {
        return jQuery.extend(true, {}, this);
    };

    this.clear = function() {
        this.hasOptions = false;
        this.options = [];
    };

}
