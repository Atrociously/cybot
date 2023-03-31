use core::cell::{OnceCell, Ref, RefCell, RefMut};

use cortex_m::interrupt::{CriticalSection, Mutex};

pub struct CriticalCell<T> {
    inner: Mutex<RefCell<T>>,
}

impl<T> CriticalCell<T> {
    #![allow(dead_code)]

    pub const fn new(v: T) -> Self {
        Self {
            inner: Mutex::new(RefCell::new(v)),
        }
    }

    pub fn borrow<'a>(&'a self, cs: &'a CriticalSection) -> Ref<'a, T> {
        self.inner.borrow(cs).borrow()
    }

    pub fn borrow_mut<'a>(&'a self, cs: &'a CriticalSection) -> RefMut<'a, T> {
        self.inner.borrow(cs).borrow_mut()
    }

    pub fn try_borrow<'a>(&'a self, cs: &'a CriticalSection) -> Option<Ref<'a, T>> {
        self.inner.borrow(cs).try_borrow().ok()
    }

    pub fn try_borrow_mut<'a>(&'a self, cs: &'a CriticalSection) -> Option<RefMut<'a, T>> {
        self.inner.borrow(cs).try_borrow_mut().ok()
    }
}

impl<T: Clone> CriticalCell<T> {
    pub fn read(&self) -> T {
        cortex_m::interrupt::free(|cs| {
            self.borrow(cs).clone()
        })
    }
}

pub struct CriticalOnce<T> {
    inner: OnceCell<T>,
}

impl<T> CriticalOnce<T> {
    pub const fn new() -> Self {
        Self {
            inner: OnceCell::new(),
        }
    }

    /// Only requires a critical section for initialization
    /// the value may live beyond the lifetime of the CriticalSection
    /// because it can no longer be changed
    pub fn get_or_init<'a>(&'a self, _cs: &CriticalSection, f: impl FnOnce() -> T) -> &'a T {
        self.inner.get_or_init(f)
    }
}

unsafe impl<T: Sync> Sync for CriticalOnce<T> {}
