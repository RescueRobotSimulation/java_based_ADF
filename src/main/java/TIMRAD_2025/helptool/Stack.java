package TIMRAD_2025.helptool;

public interface Stack<E> {
	public boolean isEmpty();

	public E peek();

	public E pop();

	public E push(E e);

	public int search(E e);

	public void clear();
}
