
export function installFailureReplay(root, overlay, { hasFailure, replay }) {
    let gesture = null;
    root.addEventListener('pointerdown', event => {
        gesture = event.button === 0 && hasFailure() && !event.target.closest('[data-g1-interactive]')
            ? { id: event.pointerId, x: event.clientX, y: event.clientY, moved: false } : null;
    }, true);
    root.addEventListener('pointermove', event => {
        if (gesture?.id === event.pointerId) {
            gesture.moved ||= Math.hypot(event.clientX-gesture.x, event.clientY-gesture.y) > 5;
        }
    }, true);
    root.addEventListener('pointercancel', () => { gesture = null; }, true);
    root.addEventListener('click', event => {
        const shouldReplay = gesture && !gesture.moved && hasFailure()
            && !event.target.closest('[data-g1-interactive]');
        gesture = null;
        if (shouldReplay) { event.preventDefault(); event.stopPropagation(); replay(); }
    }, true);
    overlay.addEventListener('keydown', event => {
        if (hasFailure() && (event.key === 'Enter' || event.key === ' ')) {
            event.preventDefault(); replay();
        }
    });
}
