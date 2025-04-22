FROM docker.1ms.run/library/ruby:3.3.4 AS base

RUN apt-get update -qq && \
    apt-get install -y --no-install-recommends \
    build-essential \
    git \
    curl \
    libjemalloc-dev \
    libvips \
    sqlite3 \
    postgresql-client \
    libpq-dev \
    nodejs

RUN gem install bundler && \
    bundle config set --local path vendor/bundle

WORKDIR /rails

FROM base AS production

COPY Gemfile Gemfile.lock /rails/

RUN bundle install

COPY . /rails/

RUN bundle exec rails assets:precompile

ENTRYPOINT ["/rails/bin/docker-entrypoint"]

HEALTHCHECK --interval=10s --timeout=5s --start-period=60s --retries=3 \
  CMD curl -f http://localhost:3000/up || exit 1

EXPOSE 3000
CMD ["bin/rails", "server", "-b", "0.0.0.0"]
