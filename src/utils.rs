use core::cell::RefCell;
use core::future::Future;
use core::pin::Pin;
use core::task::Context;
use core::task::Poll;

pub struct CancelGuard<'a, P, T, F>
where
    T: Future,
    F: FnOnce(&'a RefCell<&'a mut P>),
{
    periph: &'a RefCell<&'a mut P>,
    task: Option<T>,
    cancel_fn: Option<F>,
}

impl<'a, P, T, F> CancelGuard<'a, P, T, F>
where
    T: Future,
    F: FnOnce(&'a RefCell<&'a mut P>),
{
    pub fn new(periph: &'a RefCell<&'a mut P>, new_fut: T, cancel_fn: F) -> Self {
        Self {
            periph,
            task: Some(new_fut),
            cancel_fn: Some(cancel_fn),
        }
    }
}

impl<'a, P, T, F> Unpin for CancelGuard<'a, P, T, F>
where
    T: Future + Unpin,
    F: FnOnce(&'a RefCell<&'a mut P>),
{
}

impl<'a, P, T, F> Future for CancelGuard<'a, P, T, F>
where
    T: Future + Unpin,
    F: FnOnce(&'a RefCell<&'a mut P>),
{
    type Output = T::Output;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<T::Output> {
        use futures::FutureExt;

        if let Poll::Ready(v) = self
            .task
            .as_mut()
            .expect("cannot poll CancelGuard twice")
            .poll_unpin(cx)
        {
            // not strictly necessary but makes sure we panic if we poll after
            // completion
            self.task = None;
            self.cancel_fn = None;
            return Poll::Ready(v);
        }
        Poll::Pending
    }
}

impl<'a, P, T, F> Drop for CancelGuard<'a, P, T, F>
where
    T: Future,
    F: FnOnce(&'a RefCell<&'a mut P>),
{
    fn drop(&mut self) {
        // force the borrow to be dropped
        self.task = None;

        if let Some(cancel_fn) = self.cancel_fn.take() {
            cancel_fn(self.periph);
        }
    }
}
