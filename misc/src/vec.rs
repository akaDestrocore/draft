#![no_std]

use core::ops::{Deref, DerefMut, Index, IndexMut};
use core::marker::PhantomData;
use core::iter::FromIterator;
use core::fmt;

#[derive(Clone)]
pub struct Vec<T, const N: usize> {
    data: [T; N],
    len: usize,
}

impl<T: Copy + Default, const N: usize> Vec<T, N> {
    pub const fn new() -> Self {
        Self {
            data: [T::default(); N],
            len: 0,
        }
    }

    pub fn push(&mut self, value: T) -> bool {
        if self.len < N {
            self.data[self.len] = value;
            self.len += 1;
            true
        } else {
            false
        }
    }

    pub fn pop(&mut self) -> Option<T> {
        if self.len > 0 {
            self.len -= 1;
            Some(self.data[self.len])
        } else {
            None
        }
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn capacity(&self) -> usize {
        N
    }

    pub fn clear(&mut self) {
        self.len = 0;
    }

    pub fn truncate(&mut self, len: usize) {
        if len < self.len {
            self.len = len;
        }
    }
}

impl<T, const N: usize> Deref for Vec<T, N> {
    type Target = [T];

    fn deref(&self) -> &Self::Target {
        &self.data[0..self.len]
    }
}

impl<T, const N: usize> DerefMut for Vec<T, N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.data[0..self.len]
    }
}

impl<T, const N: usize> AsRef<[T]> for Vec<T, N> {
    fn as_ref(&self) -> &[T] {
        &self.data[0..self.len]
    }
}

impl<T, const N: usize> AsMut<[T]> for Vec<T, N> {
    fn as_mut(&mut self) -> &mut [T] {
        &mut self.data[0..self.len]
    }
}

impl<T, const N: usize> Index<usize> for Vec<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl<T, const N: usize> IndexMut<usize> for Vec<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

impl<T: fmt::Debug, const N: usize> fmt::Debug for Vec<T, N> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_list().entries(&self.data[0..self.len]).finish()
    }
}