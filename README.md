This repository is the website for rosdabbler. Access using https://rosdabbler.github.io/

Local installation:
(See https://jekyllrb.com/docs/installation/ubuntu/)

* Install Ruby and other prerequisites
```
sudo apt-get install ruby-full build-essential zlib1g-dev
```
Setup path for Ruby Gems
```
echo '# Install Ruby Gems to ~/gems' >> ~/.bashrc
echo 'export GEM_HOME="$HOME/gems"' >> ~/.bashrc
echo 'export PATH="$HOME/gems/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Install Jekyll and Bundler
```
gem install jekyll bundler
```
