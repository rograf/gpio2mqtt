module.exports = function uart() {
    return new Promise((resolve, reject) => {
        setTimeout(() => {
            resolve({ distance: 100 });
        }, 5000);
    });
};