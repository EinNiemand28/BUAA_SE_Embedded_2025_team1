class LocalesController < ApplicationController
  def switch
    locale = params[:locale].to_s.strip.to_sym
    
    if I18n.available_locales.include?(locale)
      cookies.permanent[:locale] = locale
      I18n.locale = locale
    end
    
    redirect_back(fallback_location: root_path)
  end
end 