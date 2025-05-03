FROM ruby:3.3.4-slim AS base

RUN \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm main contrib non-free-firmware" > /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm-updates main contrib non-free-firmware" >> /etc/apt/sources.list && \
    echo "deb https://mirrors.tuna.tsinghua.edu.cn/debian/ bookworm-backports main contrib non-free-firmware" >> /etc/apt/sources.list && \
    echo "deb https://security.debian.org/debian-security bookworm-security main contrib non-free-firmware" >> /etc/apt/sources.list && \
    # ----------------------

    # 更新 APT 缓存并安装依赖
    # 将更新、安装和清理放在同一个 RUN 指令中，以优化镜像层和减小大小
    apt-get update && \
    apt-get install -y --no-install-recommends \
        build-essential \
        git \
        curl \
        libjemalloc-dev \
        libvips \
        sqlite3 \
        postgresql-client \
        libpq-dev \
        nodejs \
        watchman && \
    # 清理 APT 缓存以减小镜像体积
    rm -rf /var/lib/apt/lists/*

RUN gem install bundler && \
    bundle config set --local path vendor/bundle

WORKDIR /rails

FROM base AS production

COPY Gemfile Gemfile.lock /rails/

RUN bundle config mirror.https://rubygems.org https://mirrors.tuna.tsinghua.edu.cn/rubygems/

RUN bundle config

RUN bundle install

COPY package.json yarn.lock /rails/

RUN yarn install --check-files
RUN yarn cache clean

COPY . /rails/

RUN bundle exec rails assets:precompile

ENTRYPOINT ["/rails/bin/docker-entrypoint"]

# HEALTHCHECK --interval=10s --timeout=5s --start-period=60s --retries=3 \
#   CMD curl -f http://localhost:3000/up || exit 1

EXPOSE 3000
CMD ["bin/rails", "server", "-b", "0.0.0.0"]

