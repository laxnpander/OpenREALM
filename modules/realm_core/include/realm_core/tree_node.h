/**
* This file is part of OpenREALM.
*
* Copyright (C) 2018 Alexander Kern <laxnpander at gmail dot com> (Braunschweig University of Technology)
* For more information see <https://github.com/laxnpander/OpenREALM>
*
* OpenREALM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* OpenREALM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with OpenREALM. If not, see <http://www.gnu.org/licenses/>.
*/

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
  T _data{};
  std::weak_ptr<TreeNode<T>> _parent{nullptr};
  std::vector<std::shared_ptr<TreeNode<T>>> _children{};

};

template<class T>
void TreeNode<T>::changeParent(std::weak_ptr<TreeNode<T>> parent)
{
  parent.lock()->addChild(this->shared_from_this());
  _parent.lock()->removeChild(this->shared_from_this());
  _parent = parent;
}

template<class T>
TreeNode<T>::TreeNode(T data, std::weak_ptr<TreeNode<T>> parent) : _parent(parent)
{
  _data = data;
}

template<class T>
T TreeNode<T>::getData() const
{
  return _data;
}

template<class T>
void TreeNode<T>::setData(const T &data)
{
  _data = data;
}

template<class T>
TreeNode<T> &TreeNode<T>::addChild(const T &data)
{
  _children.push_back(std::make_shared<TreeNode<T>>(data, this->shared_from_this()));
  return *_children.back();
}

template<class T>
TreeNode<T> &TreeNode<T>::addChild(std::shared_ptr<TreeNode<T>> ptr)
{
  _children.push_back(ptr);
  return *_children.back();
}

template<class T>
void TreeNode<T>::removeChild(size_t indx)
{
  _children.erase(_children.begin() + indx);
}

template<class T>
void TreeNode<T>::removeChild(std::shared_ptr<TreeNode<T>> ptr)
{
  _children.erase(std::remove(_children.begin(), _children.end(), ptr), _children.end());
}

template<class T>
const std::shared_ptr<TreeNode<T>> TreeNode<T>::findChild(const T &data) const
{
  for (size_t i{0}; i < _children.size(); ++i)
  {
    if (_children[i]->getData() == data)
    {
      return _children[i];
    }
    auto find_child = _children[i]->findChild(data);
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
  for (size_t i{0}; i < _children.size(); ++i)
  {
    if (_children[i]->getData() == data)
    {
      return _children[i];
    }
    auto find_child = _children[i]->findChild(data);
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
  assert(indx < _children.size());
  return _children[indx];
}

template<class T>
std::shared_ptr<TreeNode<T> > TreeNode<T>::getChild(size_t indx)
{
  assert(indx < _children.size());
  return _children[indx];
}

template<class T>
const std::weak_ptr<TreeNode<T> > TreeNode<T>::getParent() const
{
  return _parent;
}

template<class T>
std::weak_ptr<TreeNode<T>> TreeNode<T>::getParent()
{
  return _parent;
}


template<class T>
size_t TreeNode<T>::getNumChildren() const
{
  return _children.size();
}

template<class T>
void TreeNode<T>::print() const
{
  std::cout << _data << " ";
  if (!_children.empty())
  {
    std::cout << "(";
    for (const auto &child : _children)
    {
      child->print();
    }
    std::cout << ")";
  }
}

} // namespace realm

#endif //OPENREALM_TREE_NODE_H
