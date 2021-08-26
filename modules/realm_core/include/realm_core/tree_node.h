

#ifndef OPENREALM_TREE_NODE_H
#define OPENREALM_TREE_NODE_H

#include <iostream>
#include <vector>
#include <memory>
#include <algorithm>
#include <sstream>
#include <cassert>

namespace realm
{

// Based on an implementation by @Stellaris: https://codereview.stackexchange.com/questions/159920/c11-simple-tree-implementation

template<class T>
class TreeNode : public std::enable_shared_from_this<TreeNode<T>>
{
public:

  explicit TreeNode(T data = T(), std::weak_ptr<TreeNode<T>> parent = std::weak_ptr<TreeNode<T>>());

  T getData() const;

  void setData(const T &data);

  TreeNode<T> &addChild(const T &data);

  TreeNode<T> &addChild(std::shared_ptr<TreeNode<T>> ptr);

  void removeChild(size_t indx);

  void removeChild(std::shared_ptr<TreeNode<T>> ptr);

  const std::shared_ptr<TreeNode<T> > findChild(const T &data) const;

  std::shared_ptr<TreeNode<T> > findChild(const T &data);

  std::shared_ptr<const TreeNode<T> > getChild(size_t indx) const;

  std::shared_ptr<TreeNode<T>> getChild(size_t indx);

  const std::weak_ptr<TreeNode<T>> getParent() const;

  std::weak_ptr<TreeNode<T>> getParent();

  void changeParent(std::weak_ptr<TreeNode<T>> parent);

  bool hasChild(const T &data) const
  {
    return findChild(data) != nullptr;
  }

  size_t getNumChildren() const;

  void print() const;

private:
  T m_data{};
  std::weak_ptr<TreeNode<T>> m_parent{nullptr};
  std::vector<std::shared_ptr<TreeNode<T>>> m_children{};

};

template<class T>
void TreeNode<T>::changeParent(std::weak_ptr<TreeNode<T>> parent)
{
  parent.lock()->addChild(this->shared_from_this());
  m_parent.lock()->removeChild(this->shared_from_this());
  m_parent = parent;
}

template<class T>
TreeNode<T>::TreeNode(T data, std::weak_ptr<TreeNode<T>> parent) : m_parent(parent)
{
  m_data = data;
}

template<class T>
T TreeNode<T>::getData() const
{
  return m_data;
}

template<class T>
void TreeNode<T>::setData(const T &data)
{
  m_data = data;
}

template<class T>
TreeNode<T> &TreeNode<T>::addChild(const T &data)
{
  m_children.push_back(std::make_shared<TreeNode<T>>(data, this->shared_from_this()));
  return *m_children.back();
}

template<class T>
TreeNode<T> &TreeNode<T>::addChild(std::shared_ptr<TreeNode<T>> ptr)
{
  m_children.push_back(ptr);
  return *m_children.back();
}

template<class T>
void TreeNode<T>::removeChild(size_t indx)
{
  m_children.erase(m_children.begin() + indx);
}

template<class T>
void TreeNode<T>::removeChild(std::shared_ptr<TreeNode<T>> ptr)
{
  m_children.erase(std::remove(m_children.begin(), m_children.end(), ptr), m_children.end());
}

template<class T>
const std::shared_ptr<TreeNode<T>> TreeNode<T>::findChild(const T &data) const
{
  for (size_t i{0}; i < m_children.size(); ++i)
  {
    if (m_children[i]->data() == data)
    {
      return m_children[i];
    }
    auto find_child = m_children[i]->findChild(data);
    if (find_child != nullptr)
    {
      return find_child;
    }
  }
  return nullptr;
}

template<class T>
std::shared_ptr<TreeNode<T>> TreeNode<T>::findChild(const T &data)
{
  for (size_t i{0}; i < m_children.size(); ++i)
  {
    if (m_children[i]->data() == data)
    {
      return m_children[i];
    }
    auto find_child = m_children[i]->findChild(data);
    if (find_child != nullptr)
    {
      return find_child;
    }
  }
  return nullptr;
}

template<class T>
std::shared_ptr<const TreeNode<T>> TreeNode<T>::getChild(size_t indx) const
{
  assert(indx < m_children.size());
  return m_children[indx];
}

template<class T>
std::shared_ptr<TreeNode<T> > TreeNode<T>::getChild(size_t indx)
{
  assert(indx < m_children.size());
  return m_children[indx];
}

template<class T>
const std::weak_ptr<TreeNode<T> > TreeNode<T>::getParent() const
{
  return m_parent;
}

template<class T>
std::weak_ptr<TreeNode<T>> TreeNode<T>::getParent()
{
  return m_parent;
}


template<class T>
size_t TreeNode<T>::getNumChildren() const
{
  return m_children.size();
}

template<class T>
void TreeNode<T>::print() const
{
  std::cout << m_data << " ";
  if (!m_children.empty())
  {
    std::cout << "(";
    for (const auto &child : m_children)
    {
      child->print();
    }
    std::cout << ")";
  }
}

} // namespace realm

#endif //OPENREALM_TREE_NODE_H
