function checkPostCondition(conditionType, status) {
    if (conditionType == PostCondType.SUCCESS) {
        return status == ExecutionStatus.SUCCESS;
    } else {
        console.log('Unknown postcondition type.');
        return false;
    }
}