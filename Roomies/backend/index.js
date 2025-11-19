import getPosts from './postController.js';

getPosts().forEach(post => {
  console.log(`Post ID: ${post.id}, Title: ${post.title}`);
});

console.log(getPosts());